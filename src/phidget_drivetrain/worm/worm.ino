#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

Servo motorleft;  // Create a Servo object to control the servo motor
Servo motorright; 
const int motorPinleft = 9;  
const int motorPinright = 8; 


ros::NodeHandle nh;

// Define a publisher for debugging
std_msgs::String debug_msg;
ros::Publisher debug_pub("debug_info", &debug_msg);

void motorCommandCallback(const std_msgs::Float32& cmd_msg) {
  float command = cmd_msg.data;
  int motorSpeed = map(command * 100, -100, 100, 0, 180); 
  int motorSpeedright = map(command * 75, -100, 100, 180, 0); 


  char buf[50];
  dtostrf(command, 4, 2, buf);  // Convert float to string
  String debugStr = "Received: " + String(buf) + ", Mapped: " + String(motorSpeed);
  debug_msg.data = debugStr.c_str();
  debug_pub.publish(&debug_msg);

  motorleft.write(motorSpeed);
  motorright.write(motorSpeedright);

}

ros::Subscriber<std_msgs::Float32> sub("motor_command", motorCommandCallback);

void setup() {
  motorleft.attach(motorPinleft); 
  motorright.attach(motorPinright);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(debug_pub);
}

void loop() {
  nh.spinOnce();
  delay(10);  // Small delay for stability
}
