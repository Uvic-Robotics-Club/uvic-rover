#include <ros.h>
#include <sensor_msgs/Joy.h>

// PWM pins

//// PWM pins
//#define LEFT_BACK_PIN 3
//#define LEFT_FRONT_PIN 6
//#define RIGHT_BACK_PIN 9
//#define RIGHT_FRONT_PIN 11
//
//// Direction pins
//#define LEFT_BACK_DIR_PIN 2
//#define LEFT_FRONT_DIR_PIN 7
//#define RIGHT_BACK_DIR_PIN 8
//#define RIGHT_FRONT_DIR_PIN 13

#define LEFT_PWM 5
#define LEFT_DIR 4
#define RIGHT_PWM 3
#define RIGHT_DIR 2


#define FORWARD LOW
#define REVERSE HIGH
#define TIMEOUT_RESET_MOTORS_MILLIS 2000
#define DEADZONE 20
unsigned long last_speed_command_time_millis;

ros::NodeHandle  nh;

// callback function needed that checks if any data is incoming 
void messageCb( const sensor_msgs::Joy& joystick){

  last_speed_command_time_millis = millis();

  // X and Y axis is range [-100.0, 100.0] where negative is reverse

  int right = joystick.axes[5] * 100;
  int left = joystick.axes[2] * 100;
  int reverse = joystick.buttons[5];

  if(reverse == 1){
    resetSpeed();
    return;
  }


  int write_speed_left = map(left,100,-100,0,255);
  int write_speed_right = map(right,100,-100,0,255);

  int linear_v = (write_speed_left + write_speed_right) / 2.0;
  int angular_v = (write_speed_right - write_speed_left) / 2.0;

  int left_wheel = linear_v + angular_v;
  int right_wheel = linear_v - angular_v;
  

  digitalWrite(LEFT_DIR,0);
  digitalWrite(RIGHT_DIR,1);

  analogWrite(LEFT_PWM,left_wheel);
  analogWrite(RIGHT_PWM,right_wheel);
  

}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

// function which resets motors to 0, such that drivetrain is not mving.
void resetSpeed() {
  
//  // Write motor direction
//  digitalWrite(LEFT_BACK_DIR_PIN, 0);
//  digitalWrite(LEFT_FRONT_DIR_PIN, 0);
//  digitalWrite(RIGHT_BACK_DIR_PIN, 0);
//  digitalWrite(RIGHT_FRONT_DIR_PIN, 0);
//
//  // Write motor speed
//  analogWrite(LEFT_BACK_PIN, 0);
//  analogWrite(LEFT_FRONT_PIN, 0);
//  analogWrite(RIGHT_BACK_PIN, 0);
//  analogWrite(RIGHT_FRONT_PIN, 0);  

  digitalWrite(LEFT_DIR,0);
  digitalWrite(RIGHT_DIR,0);

  analogWrite(LEFT_PWM,0);
  analogWrite(RIGHT_PWM,0);
}


void setup() {
  // put your setup code here, to run once:
  // pinmode()
//  pinMode(LEFT_BACK_PIN, OUTPUT);
//  pinMode(LEFT_FRONT_PIN, OUTPUT);
//  pinMode(RIGHT_BACK_PIN, OUTPUT);
//  pinMode(RIGHT_FRONT_PIN, OUTPUT);
//
//  // Configure direction pins
//  pinMode(LEFT_BACK_DIR_PIN, OUTPUT);
//  pinMode(LEFT_FRONT_DIR_PIN, OUTPUT);
//  pinMode(RIGHT_BACK_DIR_PIN, OUTPUT);
//  pinMode(RIGHT_FRONT_DIR_PIN, OUTPUT);
  pinMode(LEFT_DIR,OUTPUT);
  pinMode(RIGHT_DIR,OUTPUT);

  pinMode(LEFT_PWM,OUTPUT);
  pinMode(RIGHT_PWM,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}


void loop() {

    nh.spinOnce();
    
    delay(10);

     //If timeout since last speed command, reset motors.
    if ((millis() - last_speed_command_time_millis) > TIMEOUT_RESET_MOTORS_MILLIS) {
      resetSpeed();
    }
}
