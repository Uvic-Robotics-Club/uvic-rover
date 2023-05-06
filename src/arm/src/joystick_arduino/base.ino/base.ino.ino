#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <PWM.h>

ros::NodeHandle  nh;

int base = 3; // pwm pin
int baseDir = 2;

int gripper = 9; //pwm pin
int gripperDir1 = 10;
int gripperDir2 = 12;


void messageCb( const sensor_msgs::Joy& joystick){

  // light up when sending signal
  if (joystick.axes[3] > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }


// BASE
//  we found that sending pwm of 5 works the best for the base stepper motor
 analogWrite(base,0);
 if(joystick.axes[2] > 0.5 ){
   digitalWrite(baseDir,0);
   analogWrite(base, 5);
 }

 if(joystick.axes[2] < -0.5){
   digitalWrite(baseDir,1);
   analogWrite(base, 5);
 }

  //squeeze 

 analogWrite(gripper,0);
 if(joystick.buttons[0] == 1){
  digitalWrite(gripperDir1,HIGH);
  digitalWrite(gripperDir2,LOW);
  analogWrite(gripper, 150);
  
 }

  //un-squeeze
 if(joystick.buttons[1] == 1){
  digitalWrite(gripperDir1,LOW);
  digitalWrite(gripperDir2,HIGH);
  analogWrite(gripper, 150);
  
 }

 
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

void setup() {
  // put your setup code here, to run once:
  pinMode(base,OUTPUT);
  pinMode(baseDir, OUTPUT);
  pinMode(gripper, OUTPUT);
  pinMode(gripperDir1, OUTPUT);
  pinMode(gripperDir2, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
