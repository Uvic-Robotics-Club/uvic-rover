#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

// Pin definitions for motor control
const int LEFT_MOTOR_PWM_PIN = 5;   // PWM pin for left motor
const int LEFT_MOTOR_DIR_PIN = 4;   // Direction pin for left motor
const int RIGHT_MOTOR_PWM_PIN = 6;  // PWM pin for right motor
const int RIGHT_MOTOR_DIR_PIN = 7;  // Direction pin for right motor

ros::NodeHandle nh;

// Motor control variables
int left_pwm = 0;
bool left_dir = true;
int right_pwm = 0;
bool right_dir = true;

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
}

void loop() {
  nh.spinOnce();
  delay(1);
}