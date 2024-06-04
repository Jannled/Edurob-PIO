#define ROS_EN

#ifdef ROS_EN
#define RCCHECK(fn)              \
	{                              \
		rcl_ret_t temp_rc = fn;      \
		if ((temp_rc != RCL_RET_OK)) \
		{                            \
			error_loop();              \
		}                            \
	}
#define RCSOFTCHECK(fn)          \
	{                              \
		rcl_ret_t temp_rc = fn;      \
		if ((temp_rc != RCL_RET_OK)) \
		{                            \
		}                            \
	}
#endif // ROS_EN

#include <WiFi.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "math.h"

#include <dcpwm.h>       // PWM
#include <ESP_Counter.h> // Encoder
#include <AutoPID.h>     // PID Library https://github.com/r-downing/AutoPID
#include <Eigen.h>       // Linear math
#include <Eigen/QR>      // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;   // Eigen related statement; simplifies syntax for declaration of matrices

// ROS specific includes
#ifdef ROS_EN

// Wifi and config related includes
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
#include "SPIFFS.h"
#include "cpp/INIReader.h"
#include "lwip/inet.h"
#endif // MICRO_ROS_TRANSPORT_ARDUINO_WIFI

#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// ROS Messages
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#endif // ROS_EN

// Project specific headers
#include "parameter.h"

// kinematik header
#include "kinematik.h"


// Hardware
static ESP_Counter WheelEncoder[NumMotors];      // Hardware-Encoder-Units
static DCPWM MotorPWM[NumMotors];                // Hardware-PWM-Units
static AutoPID *speedController[NumMotors];      // PID-Units
static pidParam speedControllerParam[NumMotors]; // PID-Parameter

// Variables
int64_t encoderOld[NumMotors];       // Last encoder values
int64_t encoderNew[NumMotors];       // Current encoder values
const int windowSize = 5;            // Number of values in moving average window
float inputs[NumMotors][windowSize]; // Input values for moving average
float input_sum[NumMotors];           // Sum of values for moving average
int windowIndex = 0;                 // Currently accessed cell of inputs[motorNum][x];

// Setpoints
static double setpointSpeed[NumMotors]; // Motor speed setpoints

// Matrix
MatrixXd kinematik(4, 3);    // Kinematics matrix
MatrixXd kinematikInv(3, 4); // Inverse Kinematics matrix

Vector3d target_robot_speed; 	// Vector with translational and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))
Vector4d target_wheel_speed;    // Target Wheel speeds in rad/s
Vector4d target_odom_speed;     // Target Odometry Speed

Vector3d current_robot_speed;         // Vector with translational and rotational robot speeds in m/s and rad/s  (X(m/s), Y(m/s), Z(rad/s))

double rosQuaternion[4];     // Quaternion for ROS transform message

#ifdef ROS_EN
// ROS
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

geometry_msgs__msg__TransformStamped tfData[3];
geometry_msgs__msg__TransformStamped tfStaticData[3];
rcl_subscription_t twistSubscriber;
geometry_msgs__msg__Twist twistMessage;

tf2_msgs__msg__TFMessage messageTf;
rcl_publisher_t publisherTf;
tf2_msgs__msg__TFMessage messageTfStatic;
rcl_publisher_t publisherTfStatic;
rcl_publisher_t publisherSpeed;
#endif // ROS_EN

// Error function in case of unhandled ros-error
void error_loop()
{
	while (1)
	{
		Serial.println("ERR");
		delay(100);
	}
}

// Convert Euler degrees to quaternion
const void euler_to_quat(float x, float y, float z, double *q)
{
	float c1 = cos(y / 2.0);
	float c2 = cos(z / 2.0);
	float c3 = cos(x / 2.0);

	float s1 = sin(y / 2.0);
	float s2 = sin(z / 2.0);
	float s3 = sin(x / 2.0);

	q[0] = c1 * c2 * c3 - s1 * s2 * s3;
	q[1] = s1 * s2 * c3 + c1 * c2 * s3;
	q[2] = s1 * c2 * c3 + c1 * s2 * s3;
	q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

// Rotate 2D-Vector
const void rotate2D(float r, double &x, double &y)
{
	double temp = x * cos(r) - y * sin(r);
	y = x * sin(r) + y * cos(r);
	x = temp;
}

#ifdef ROS_EN
// Twist message cb
void subscription_callback(const void *msg_in)
{
	const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
	target_robot_speed << msg->linear.x, msg->linear.y, msg->angular.z;
}

#endif //ROS_EN

bool initROS2()
{
	allocator = rcl_get_default_allocator();
	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	// create node
	RCCHECK(rclc_node_init_default(&node, "Edurob", "", &support));
	// create tf publisher
	RCCHECK(rclc_publisher_init_default(
			&publisherTf,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
			"/tf"));

	// create tf publisher
	// Set publisher QoS
	rmw_qos_profile_t rmw_qos_profile_tfstatic = {RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10U, RMW_QOS_POLICY_RELIABILITY_RELIABLE, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL, {0ULL, 0ULL}, {0ULL, 0ULL}, RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT, {0ULL, 0ULL}, false};

	RCCHECK(rclc_publisher_init(
			&publisherTfStatic,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
			"/tf_static",
			&rmw_qos_profile_tfstatic));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
			&twistSubscriber,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			"cmd_vel"));

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &twistSubscriber, &twistMessage, &subscription_callback, ON_NEW_DATA));

	return true;
}

// Init Encoder- and PWM-Units
bool initHardware()
{
	pinMode(EnablePIN, OUTPUT);   // Motor enable signal
	digitalWrite(EnablePIN, LOW); // Disable motors
	for (int i = 0; i < NumMotors; i++)
	{
		if (!WheelEncoder[i].init(i, Enc_A[i], Enc_B[i]))
			return false;
	}

	int currentChannel = 0;
	for (int i = 0; i < NumMotors; i++)
	{
		if (!MotorPWM[i].init(30000, 10, currentChannel, currentChannel + 1, PWM_A[i], PWM_B[i]))
			return false;
		
		MotorPWM[i].setPWM(0); // Set initial speed to 0
		currentChannel += 2;
	}

	return true;
}

// Init speed controller
void initPID()
{
	for (int i = 0; i < NumMotors; i++)
	{
		speedControllerParam[i].input = 0;
		speedControllerParam[i].output = 0;
		speedControllerParam[i].setpoint = 0;
		speedControllerParam[i].p = 0.8;
		speedControllerParam[i].i = 0.0;
		speedControllerParam[i].d = 0.0;
		speedControllerParam[i].sampleTimeMs = sampleTime;
		speedControllerParam[i].outMin = -100.0; // %
		speedControllerParam[i].outMax = +100.0; // %

		speedController[i] = new AutoPID(
			&speedControllerParam[i].input, 
			&speedControllerParam[i].setpoint, 
			&speedControllerParam[i].output, 
			speedControllerParam[i].outMin, 
			speedControllerParam[i].outMax, 
			speedControllerParam[i].p, 
			speedControllerParam[i].i, 
			speedControllerParam[i].d
		);

		speedController[i]->setTimeStep(speedControllerParam[i].sampleTimeMs);
		speedController[i]->setBangBang(0, 0); // Disable BangBang-Control
	}
}

// Init platform specific kinematics
void initMatrix()
{
#if defined(MECANUM) // Mecanum
	mecanum_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values

#elif defined(DIFF) // Diff
	differential_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values

#elif defined(OMNI4) // Omni 4 Wheels
	omni_4_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values

#elif defined(OMNI3) // Omni 3 Wheels
	omni_3_matrix(kinematik, kinematikInv, l1, l2); // sets the kinematik and kinematikInv to the desired values
#endif
}


void task_drive()
{
	target_wheel_speed = (1 / wheelRadius) * kinematik * target_robot_speed; // rad/s
	target_odom_speed = target_wheel_speed / incrementsToRad; // increments/s

	// Read
	for(int i = 0; i < NumMotors; i++)
	{
		// Store old encoder values and read new ones
		encoderOld[i] = encoderNew[i];
		encoderNew[i] = EncoderDir[i] * WheelEncoder[i].getCount();

		const double measured_odom = (encoderNew[i] - encoderOld[i]) / (millis()/1000u);
		const double measured_duty = measured_odom * Rad2PWM;

		speedControllerParam[i].input = min(100.0, max(-100.0, measured_duty));

		const double target_duty = target_odom_speed[i] * Rad2PWM;
		speedControllerParam[i].setpoint = min(100.0, max(-100.0, target_duty)); // dutycycle in %
		// Serial.print(target_wheel_speed[i]);
		// Serial.print(" -> ");
		// Serial.print(target_odom_speed[i]);
		// Serial.print(" -> ");
		// Serial.println(target_duty);
	}

	// Plan
	for(int i = 0; i < NumMotors; i++)
		speedController[i]->run();

	// Act
	for(int i = 0; i < NumMotors; i++)
		MotorPWM[i].setPWM(MotorDir[i] * speedControllerParam[i].output);

	Serial.printf(">i1:%.4f\n", speedControllerParam[0].input);
	Serial.printf(">i2:%.4f\n", speedControllerParam[1].input);
	Serial.printf(">i3:%.4f\n", speedControllerParam[2].input);
	Serial.printf(">i4:%.4f\n", speedControllerParam[3].input);

	Serial.printf(">o1:%.4f\n", speedControllerParam[0].output);
	Serial.printf(">o2:%.4f\n", speedControllerParam[1].output);
	Serial.printf(">o3:%.4f\n", speedControllerParam[2].output);
	Serial.printf(">o4:%.4f\n", speedControllerParam[3].output);
}


#ifdef ROS_EN
// Set tf data
void setTfData()
{
	tfData[0].header.frame_id =
			micro_ros_string_utilities_set(tfData[0].header.frame_id, "/odom");
	tfData[0].child_frame_id =
			micro_ros_string_utilities_set(tfData[0].child_frame_id, "/base_link");
	tfData[0].transform.translation.x = target_odom_speed[0];
	tfData[0].transform.translation.y = target_odom_speed[1];
	euler_to_quat(0, 0, target_odom_speed[2], rosQuaternion);
	tfData[0].transform.rotation.x = (double)rosQuaternion[1];
	tfData[0].transform.rotation.y = (double)rosQuaternion[2];
	tfData[0].transform.rotation.z = (double)rosQuaternion[3];
	tfData[0].transform.rotation.w = (double)rosQuaternion[0];
	tfData[0].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
	tfData[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

	messageTf.transforms.size = 1;
	messageTf.transforms.data = tfData;
}

// Set static tf data
void setTfStaticData()
{
	tfStaticData[0].header.frame_id =
			micro_ros_string_utilities_set(tfStaticData[0].header.frame_id, "/base_link");
	tfStaticData[0].child_frame_id =
			micro_ros_string_utilities_set(tfStaticData[0].child_frame_id, "/laser_link");
	tfStaticData[0].transform.translation.x = 0;
	tfStaticData[0].transform.translation.y = 0;
	tfStaticData[0].transform.translation.z = 0.08;
	tfStaticData[0].transform.rotation.x = 0;
	tfStaticData[0].transform.rotation.y = 0;
	tfStaticData[0].transform.rotation.z = 0;
	tfStaticData[0].transform.rotation.w = 1;
	tfStaticData[0].header.stamp.nanosec = rmw_uros_epoch_millis() * 1000;
	tfStaticData[0].header.stamp.sec = rmw_uros_epoch_millis() / 1000;

	messageTfStatic.transforms.size = 1;
	messageTfStatic.transforms.data = tfStaticData;
}
#endif //ROS_EN

// Setup
void setup()
{
	Serial.begin(115200);
	initHardware();
	initPID();
	initMatrix();

#ifdef ROS_EN
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
	#define CONFIG_FNAME "/config.ini"

	// Read the config.ini file from spiffs
	log_i("Reading " CONFIG_FNAME);
	if(!SPIFFS.begin(true))
	{
		log_i("Unable to communicate with SPIFFS.");
		return;
	}

	File f = SPIFFS.open(CONFIG_FNAME);
	char buff[512];
	f.readBytes(buff, sizeof(buff));

	INIReader reader(buff, sizeof(buff));
	if(reader.ParseError() != 0)
		log_w("Invalid line in config: %d", reader.ParseError());

	std::string ssid = reader.GetString("edurob", "SSID", "INVALID");
	std::string passwd = reader.GetString("edurob", "passwd", "NOTAPASSWORD");
	std::string agent_ip = reader.GetString("edurob", "agent_ip", "192.168.1.1");
	int64_t port = reader.GetInteger64("edurob", "client_port", 8888);
	
	uint8_t ipaddr[4] = {0};
	uint8_t i = 0;

	// Connect to uROS Agent via WiFi
	log_i("Connecting via Wifi...");
	set_microros_wifi_transports(
		(char*) ssid.c_str(), (char*) passwd.c_str(), 
		IPAddress(inet_addr(agent_ip.c_str())), port
	);
	Serial.print("Connected via Wifi (IP: ");
	Serial.print(WiFi.localIP());
	Serial.println(")");
	delay(2000);
#endif // MICRO_ROS_TRANSPORT_ARDUINO_WIFI
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_SERIAL
	set_microros_transports();
#endif // MICRO_ROS_TRANSPORT_ARDUINO_SERIAL

	initROS2();

#endif // ROS_EN
}

void loop()
{
	digitalWrite(EnablePIN, HIGH); // Enable motors

	// #############-USER-CODE-START-#####################
	// double tx = 0.0, ty = 0.0, theta = 0.0;
	// robotSpeedSetpoint << tx, ty, theta;

	// #############-USER-CODE-END-#####################

#ifdef ROS_EN

	rmw_uros_sync_session(100); // Synchronize time

	if (rmw_uros_epoch_synchronized())
	{
		setTfData();
		setTfStaticData();
		RCSOFTCHECK(rcl_publish(&publisherTf, &messageTf, NULL));
		RCSOFTCHECK(rcl_publish(&publisherTf, &messageTfStatic, NULL));
	}

	RCCHECK(rclc_executor_spin_some(&executor, 500));
	task_drive();
#endif // ROS_EN
	delay(10);
}