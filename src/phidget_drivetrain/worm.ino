#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>

Servo motor;  // Create a Servo object to control the motor
const int motorPin = 9;  // Pin connected to the motor controller

ros::NodeHandle nh;

void motorCommandCallback(const std_msgs::Float32& cmd_msg) {
  float command = cmd_msg.data;
  int motorAngle;

  if (command == 0.0) {
    motorAngle = 90;  // 0-degree position
  } else {
    motorAngle = map(command, -1.0, 1.0, 45, 135);  // Map the command to motor angle
  }

  motor.write(motorAngle);
}

ros::Subscriber<std_msgs::Float32> sub("motor_command", motorCommandCallback);

void setup() {
  motor.attach(motorPin);  // Attach the motor controller to the motorPin
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);  // Small delay for stability
}
