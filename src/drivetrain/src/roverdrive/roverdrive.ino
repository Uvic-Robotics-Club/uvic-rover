#include <ros.h>
#include <sensor_msgs/Joy.h>

// PWM pins

// PWM pins
#define LEFT_BACK_PIN 3
#define LEFT_FRONT_PIN 6
#define RIGHT_BACK_PIN 9
#define RIGHT_FRONT_PIN 11

// Direction pins
#define LEFT_BACK_DIR_PIN 2
#define LEFT_FRONT_DIR_PIN 7
#define RIGHT_BACK_DIR_PIN 8
#define RIGHT_FRONT_DIR_PIN 13


#define FORWARD LOW
#define REVERSE HIGH
#define TIMEOUT_RESET_MOTORS_MILLIS 2000
#define DEADZONE = 0.2
unsigned long last_speed_command_time_millis;

ros::NodeHandle  nh;
ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

// callback function needed that checks if any data is incoming 
void messageCb( const sensor_msgs::Joy& joystick){

  last_speed_command_time_millis = millis();


  // X and Y axis is range [-100.0, 100.0] where negative is reverse

  int x_axis = joystick.axes[3] * 100;
  int y_axis = joystick.axes[4] * 100;
  int z_button = joystick.buttons[5];

  int speedLeft;
  int speedRight;

  speedLeft = y_axis;
  speedRight = y_axis;

  if (abs(x_axis) < DEADZONE && abs(y_axis) < DEADZONE){

    if(z_button == 1 ){
      speedLeft += 50;
      speedRight -= 50;
    }
  }else{
    if(x_axis != 0){
      speedLeft += x_axis;
      speedRight -= x_axis;
    }
  }


  if(speedLeft < 0){
    speedLeft += 1;
  }else if(speedLeft > 0){
    speedLeft -= 1;
  }

  if(speedRight < 0){
    speedRight += 1;
  }else if(speedRight > 0){
    speedRight -= 1;
  }



  int write_direction_left = 0;
  int write_direction_right = 0;
  int write_speed_right = 0;
  int write_speed_left = 0;


  if(speedLeft > 0){
    write_direction_left = 0;
  }else{
    write_direction_left = 1;
  }

  if(speedRight > 0){
    write_direction_right = 0;
  }else{
    write_direction_right = 1;
  }

  write_speed_left = map(abs(speedLeft),0,100,0,255);
  write_speed_right = map(abs(speedRight),0,100,0,255);

  digitalWrite(LEFT_BACK_DIR_PIN,write_direction_left);
  digitalWrite(LEFT_FRONT_DIR_PIN,write_direction_left);
  digitalWrite(RIGHT_BACK_DIR_PIN,write_direction_right);
  digitalWrite(RIGHT_FRONT_DIR_PIN,write_direction_right);


  analogWrite(LEFT_BACK_PIN,write_speed_left);
  analogWrite(LEFT_FRONT_PIN,write_speed_left);
  analogWrite(RIGHT_BACK_PIN,write_speed_right);
  analogWrite(RIGHT_FRONT_PIN,write_speed_right);

}

// function which resets motors to 0, such that drivetrain is not mving.
void resetSpeed() {
  
  // Write motor direction
  digitalWrite(LEFT_BACK_DIR_PIN, 0);
  digitalWrite(LEFT_FRONT_DIR_PIN, 0);
  digitalWrite(RIGHT_BACK_DIR_PIN, 0);
  digitalWrite(RIGHT_FRONT_DIR_PIN, 0);

  // Write motor speed
  analogWrite(LEFT_BACK_PIN, 0);
  analogWrite(LEFT_FRONT_PIN, 0);
  analogWrite(RIGHT_BACK_PIN, 0);
  analogWrite(RIGHT_FRONT_PIN, 0);  
}


void setup() {
  // put your setup code here, to run once:
  // pinmode()
  pinMode(LEFT_BACK_PIN, OUTPUT);
  pinMode(LEFT_FRONT_PIN, OUTPUT);
  pinMode(RIGHT_BACK_PIN, OUTPUT);
  pinMode(RIGHT_FRONT_PIN, OUTPUT);

  // Configure direction pins
  pinMode(LEFT_BACK_DIR_PIN, OUTPUT);
  pinMode(LEFT_FRONT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_BACK_DIR_PIN, OUTPUT);
  pinMode(RIGHT_FRONT_DIR_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}


void loop() {
  // put your main code here, to run repeatedly:
    nh.spinOnce();
    delay(100);

    // If timeout since last speed command, reset motors.
    if ((millis() - last_speed_command_time_millis) > TIMEOUT_RESET_MOTORS_MILLIS) {
      resetSpeed();
    }
}
