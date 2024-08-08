#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <Encoder.h>

// Pin definitions for motor control
const int LEFT_MOTOR_PWM_PIN = 5;   // PWM pin for left motor
const int LEFT_MOTOR_DIR_PIN = 4;   // Direction pin for left motor
const int RIGHT_MOTOR_PWM_PIN = 6;  // PWM pin for right motor
const int RIGHT_MOTOR_DIR_PIN = 7;  // Direction pin for right motor

// Pin definitions for encoders
const int LEFT_ENCODER_PIN_A = 2;   // Left encoder pin A
const int LEFT_ENCODER_PIN_B = 3;   // Left encoder pin B
const int RIGHT_ENCODER_PIN_A = 18; // Right encoder pin A (A4 on Mega)
const int RIGHT_ENCODER_PIN_B = 19; // Right encoder pin B (A5 on Mega)

ros::NodeHandle nh;

// Motor control variables
int left_pwm = 0;
bool left_dir = true;
int right_pwm = 0;
bool right_dir = true;

// Encoder objects
Encoder leftEncoder(LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B);
Encoder rightEncoder(RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B);

// ROS message objects for publishing encoder values
std_msgs::Int32 left_encoder_msg;
std_msgs::Int32 right_encoder_msg;

// ROS publishers for encoder values
ros::Publisher left_encoder_pub("left_encoder", &left_encoder_msg);
ros::Publisher right_encoder_pub("right_encoder", &right_encoder_msg);

// Callback functions for subscribers
void leftPwmCallback(const std_msgs::Int16& msg) {
  left_pwm = msg.data;
  analogWrite(LEFT_MOTOR_PWM_PIN, left_pwm);
}

void leftDirCallback(const std_msgs::Bool& msg) {
  left_dir = msg.data;
  digitalWrite(LEFT_MOTOR_DIR_PIN, left_dir ? HIGH : LOW);
}

void rightPwmCallback(const std_msgs::Int16& msg) {
  right_pwm = msg.data;
  analogWrite(RIGHT_MOTOR_PWM_PIN, right_pwm);
}

void rightDirCallback(const std_msgs::Bool& msg) {
  right_dir = msg.data;
  digitalWrite(RIGHT_MOTOR_DIR_PIN, right_dir ? HIGH : LOW);
}

// ROS subscribers
ros::Subscriber<std_msgs::Int16> left_pwm_sub("worm_gear_front_left_pwm", &leftPwmCallback);
ros::Subscriber<std_msgs::Bool> left_dir_sub("worm_gear_front_left_dir", &leftDirCallback);
ros::Subscriber<std_msgs::Int16> right_pwm_sub("worm_gear_front_right_pwm", &rightPwmCallback);
ros::Subscriber<std_msgs::Bool> right_dir_sub("worm_gear_front_right_dir", &rightDirCallback);

void setup() {
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  
  // Initialize ROS node
  nh.initNode();
  
  // Subscribe to topics
  nh.subscribe(left_pwm_sub);
  nh.subscribe(left_dir_sub);
  nh.subscribe(right_pwm_sub);
  nh.subscribe(right_dir_sub);

  // Advertise encoder topics
  nh.advertise(left_encoder_pub);
  nh.advertise(right_encoder_pub);
}

void loop() {
  // Read encoder values
  long left_encoder_value = leftEncoder.read();
  long right_encoder_value = rightEncoder.read();

  // Publish encoder values
  left_encoder_msg.data = left_encoder_value;
  right_encoder_msg.data = right_encoder_value;
  left_encoder_pub.publish(&left_encoder_msg);
  right_encoder_pub.publish(&right_encoder_msg);

  nh.spinOnce();
  delay(10); // Small delay to control publish rate (100 Hz)
}