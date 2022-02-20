//#define USE_USBCON
// rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=9600
// This program's only purpose is to recieve the Speed and direction data from 
// the Speed custom message and writes it on the correct pins
// All pin information should be on the github wiki


#include <ros.h>
#include "runt_rover/Speed.h"

// PWM pins
#define LEFT_BACK_PIN 3
#define LEFT_MIDDLE_PIN 5
#define LEFT_FRONT_PIN 6
#define RIGHT_BACK_PIN 9
#define RIGHT_MIDDLE_PIN 10
#define RIGHT_FRONT_PIN 11

// Direction pins
#define LEFT_BACK_DIR_PIN 2
#define LEFT_MIDDLE_DIR_PIN 4
#define LEFT_FRONT_DIR_PIN 7
#define RIGHT_BACK_DIR_PIN 8
#define RIGHT_MIDDLE_DIR_PIN 12
#define RIGHT_FRONT_DIR_PIN 13

#define FORWARD LOW
#define REVERSE HIGH

// callback function needed that checks if any data is incoming 
void handleSpeed(const runt_rover::Speed speed_direction){
  
  int write_direction_left = speed_direction.leftdirection;
  int write_direction_right = speed_direction.rightdirection;
  int write_speed_left = speed_direction.leftspeed;
  int write_speed_right = speed_direction.rightspeed;

    // Write motor direction
  digitalWrite(LEFT_BACK_DIR_PIN, write_direction_left);
  digitalWrite(LEFT_MIDDLE_DIR_PIN, write_direction_left);
  digitalWrite(LEFT_FRONT_DIR_PIN, write_direction_left);
  digitalWrite(RIGHT_BACK_DIR_PIN, write_direction_right);
  digitalWrite(RIGHT_MIDDLE_DIR_PIN, write_direction_right);
  digitalWrite(RIGHT_FRONT_DIR_PIN, write_direction_right);

  // Write motor speed
  analogWrite(LEFT_BACK_PIN, write_speed_left);
  analogWrite(LEFT_MIDDLE_PIN, write_speed_left);
  analogWrite(LEFT_FRONT_PIN, write_speed_left);
  analogWrite(RIGHT_BACK_PIN, write_speed_right);
  analogWrite(RIGHT_MIDDLE_PIN, write_speed_right);
  analogWrite(RIGHT_FRONT_PIN, write_speed_right);

    
}



//Initalize ros node and subscriber to 'data' with handleback as callback function
ros::NodeHandle nh;
ros::Subscriber<runt_rover::Speed> sub("drive_train",handleSpeed);

void setup()
{
  // This command sets the baud rate to 9600 and is only necessary for the arduino nano 
  nh.getHardware()->setBaud(9600); 
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(LEFT_MIDDLE_PIN, OUTPUT);
  pinMode(LEFT_FRONT_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_MIDDLE_PIN, OUTPUT);
  pinMode(RIGHT_FRONT_PIN, OUTPUT);

  // Configure direction pins
  pinMode(LEFT_BACK_DIR_PIN, OUTPUT);
  pinMode(LEFT_MIDDLE_DIR_PIN, OUTPUT);
  pinMode(LEFT_FRONT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_BACK_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MIDDLE_DIR_PIN, OUTPUT);
  pinMode(RIGHT_FRONT_DIR_PIN, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
}

void loop()
{
    nh.spinOnce();
}
