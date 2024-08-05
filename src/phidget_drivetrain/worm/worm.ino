#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

Servo motor;  // Create a Servo object to control the servo motor
const int motorPin = 9;  // Pin connected to the servo signal pin

ros::NodeHandle nh;

// Define a publisher for debugging
std_msgs::String debug_msg;
ros::Publisher debug_pub("debug_info", &debug_msg);

void motorCommandCallback(const std_msgs::Float32& cmd_msg) {
  float command = cmd_msg.data;
  int motorSpeed = map(command * 100, -100, 100, 0, 180);  // Map the command to motor speed

  // Create a debug message
  char buf[50];
  dtostrf(command, 4, 2, buf);  // Convert float to string
  String debugStr = "Received: " + String(buf) + ", Mapped: " + String(motorSpeed);
  debug_msg.data = debugStr.c_str();
  debug_pub.publish(&debug_msg);

  motor.write(motorSpeed);
}

ros::Subscriber<std_msgs::Float32> sub("motor_command", motorCommandCallback);

void setup() {
  motor.attach(motorPin);  // Attach the servo object to the specified pin
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug_pub);
}

void loop() {
  nh.spinOnce();
  delay(10);  // Small delay for stability
}
