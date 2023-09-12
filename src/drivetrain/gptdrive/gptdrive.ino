#include <ros.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle nh;

// Define motor control pins
const int motorFrontLeftSpeedPin = 4;    // Motor speed control (PWM)
const int motorFrontLeftDirPin = 33;      // Motor direction control

const int motorFrontRightSpeedPin = 5;   // Motor speed control (PWM)
const int motorFrontRightDirPin = 32;     // Motor direction control

const int motorRearLeftSpeedPin = 2;     // Motor speed control (PWM)
const int motorRearLeftDirPin = 35;      // Motor direction control

const int motorRearRightSpeedPin = 3;   // Motor speed control (PWM)
const int motorRearRightDirPin = 34;     // Motor direction control

// Define joystick values
float joyX, joyY;

// Define deadzone threshold
const float deadzone = 0.2;

// Define button index for reset
const int resetButtonIndex = 4;  // Replace with the actual button index

bool shouldReset = false;

void joyCallback(const sensor_msgs::Joy &joy_msg) {
  // Store joystick values with deadzone
  joyX = abs(joy_msg.axes[0]) > deadzone ? joy_msg.axes[0] : 0;
  joyY = abs(joy_msg.axes[1]) > deadzone ? joy_msg.axes[1] : 0;

  // Check if reset button is pressed
  shouldReset = joy_msg.buttons[resetButtonIndex] == 1;
}

void setup() {
  nh.initNode();
  ros::Subscriber<sensor_msgs::Joy> sub("j1", &joyCallback);
  nh.subscribe(sub);

  // Initialize motor control pins
  pinMode(motorFrontLeftSpeedPin, OUTPUT);
  pinMode(motorFrontLeftDirPin, OUTPUT);
  
  pinMode(motorFrontRightSpeedPin, OUTPUT);
  pinMode(motorFrontRightDirPin, OUTPUT);
  
  pinMode(motorRearLeftSpeedPin, OUTPUT);
  pinMode(motorRearLeftDirPin, OUTPUT);
  
  pinMode(motorRearRightSpeedPin, OUTPUT);
  pinMode(motorRearRightDirPin, OUTPUT);
}

void setMotorSpeedAndDirection(int speedPin, int dirPin, int speed, int direction) {
  digitalWrite(dirPin, direction);
  analogWrite(speedPin, speed);
}

void loop() {
  // Check if reset button is pressed
  if (shouldReset) {
    joyX = 0;
    joyY = 0;
    shouldReset = false;
  }

  // Convert joystick values to PWM signals
  int motorSpeedFrontLeft = joyY * 255;
  int motorSpeedFrontRight = joyY * 255;
  int motorSpeedRearLeft = joyY * 255;
  int motorSpeedRearRight = joyY * 255;

  // Determine motor directions based on joystick input
  int motorDirFrontLeft = joyY > 0 ? HIGH : LOW;
  int motorDirFrontRight = joyY > 0 ? HIGH : LOW;
  int motorDirRearLeft = joyY > 0 ? HIGH : LOW;
  int motorDirRearRight = joyY > 0 ? HIGH : LOW;

  // Apply steering control
  motorSpeedFrontLeft += joyX * 100;
  motorSpeedFrontRight -= joyX * 100;
  motorSpeedRearLeft -= joyX * 100;
  motorSpeedRearRight += joyX * 100;

  // Clip motor speeds to valid range
  motorSpeedFrontLeft = constrain(motorSpeedFrontLeft, -255, 255);
  motorSpeedFrontRight = constrain(motorSpeedFrontRight, -255, 255);
  motorSpeedRearLeft = constrain(motorSpeedRearLeft, -255, 255);
  motorSpeedRearRight = constrain(motorSpeedRearRight, -255, 255);

  // Apply PWM signals and directions to motor control pins
  setMotorSpeedAndDirection(motorFrontLeftSpeedPin, motorFrontLeftDirPin, abs(motorSpeedFrontLeft), motorDirFrontLeft);
  setMotorSpeedAndDirection(motorFrontRightSpeedPin, motorFrontRightDirPin, abs(motorSpeedFrontRight), motorDirFrontRight);
  setMotorSpeedAndDirection(motorRearLeftSpeedPin, motorRearLeftDirPin, abs(motorSpeedRearLeft), motorDirRearLeft);
  setMotorSpeedAndDirection(motorRearRightSpeedPin, motorRearRightDirPin, abs(motorSpeedRearRight), motorDirRearRight);

  nh.spinOnce();
  delay(100);
}
