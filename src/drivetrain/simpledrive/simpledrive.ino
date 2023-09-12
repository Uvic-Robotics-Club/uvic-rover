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

#define LEFT_FRONT_PWM 4
#define LEFT_FRONT_DIR 33

#define RIGHT_FRONT_PWM 5
#define RIGHT_FRONT_DIR 32


#define LEFT_BACK_PWM 2
#define LEFT_BACK_DIR 35
#define RIGHT_BACK_PWM 3
#define RIGHT_BACK_DIR 34


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

  float right = joystick.axes[4] * 100;
  float left = joystick.axes[1] * 100;
  int reverse = joystick.buttons[5];
  int stop_move = joystick.buttons[4];
  
  
  if(stop_move == 1){
    resetSpeed();
    return;
  }


  float write_speed_left = map(left,0,100,0,255);
  float write_speed_right = map(right,0,100,0,255);

  float linear_v = (write_speed_left + write_speed_right) / 2.0;
  float angular_v = (write_speed_right - write_speed_left) / 2.0;

  float left_wheel = linear_v + angular_v;
  float right_wheel = linear_v - angular_v;
  

  digitalWrite(LEFT_BACK_DIR, not reverse);
  digitalWrite(RIGHT_BACK_DIR, not reverse);

  analogWrite(LEFT_BACK_PWM,write_speed_left);
  analogWrite(RIGHT_BACK_PWM,write_speed_right);

  digitalWrite(LEFT_FRONT_DIR,not reverse);
  digitalWrite(RIGHT_FRONT_DIR,not reverse);

  analogWrite(LEFT_FRONT_PWM,write_speed_left);
  analogWrite(RIGHT_FRONT_PWM,write_speed_right);
  

}

ros::Subscriber<sensor_msgs::Joy> sub("j1", &messageCb );

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


  analogWrite(LEFT_BACK_PWM,0);
  analogWrite(RIGHT_BACK_PWM,0);

  analogWrite(LEFT_FRONT_PWM,0);
  analogWrite(RIGHT_FRONT_PWM,0);
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
  pinMode(LEFT_BACK_DIR,OUTPUT);
  pinMode(RIGHT_BACK_DIR,OUTPUT);

  pinMode(LEFT_BACK_PWM,OUTPUT);
  pinMode(RIGHT_BACK_PWM,OUTPUT);

  pinMode(LEFT_FRONT_DIR,OUTPUT);
  pinMode(RIGHT_FRONT_DIR,OUTPUT);

  pinMode(LEFT_FRONT_PWM,OUTPUT);
  pinMode(RIGHT_FRONT_PWM,OUTPUT);


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
