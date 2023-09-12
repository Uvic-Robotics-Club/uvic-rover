
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <PWM.h>

ros::NodeHandle  nh;

int32_t frequency_shoulder = 10000;
int32_t frequency_elbow = 10000;
int shoulder = 3; // pwm pin
int shoulderDir = 2;
int elbow = 10; // pwm pin
int elbowDir = 12;
//int base = 9; // pwm pin
//int baseDir = 7;


void messageCb( const sensor_msgs::Joy& joystick){
  frequency_shoulder = abs(map(joystick.axes[1]*100, -100, 100, -30000, 30000));
  frequency_elbow = abs(map(joystick.axes[5]*100, -100, 100, -30000, 30000));

  SetPinFrequencySafe(shoulder, frequency_shoulder);
  SetPinFrequencySafe(elbow, frequency_elbow);

  // light up when sending signal
  if (joystick.axes[3] > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }


  // SHOULDER
  pwmWrite(shoulder, 0);
  if (joystick.axes[1] > 0) {
    digitalWrite(shoulderDir, 0);
    pwmWrite(shoulder, 155);
  }
  if (joystick.axes[1] < 0) {
    digitalWrite(shoulderDir, 1);
    pwmWrite(shoulder, 155);
  }
 // ELBOW
 pwmWrite(elbow,0);
 if(joystick.axes[5] > 0 ){
   digitalWrite(elbowDir,0);
   pwmWrite(elbow, 155);
 }
 if(joystick.axes[5] < 0){
   digitalWrite(elbowDir,1);
   pwmWrite(elbow, 155);
 }


 //BASE
 // we found that sending pwm of 5 works the best for the base stepper motor
// pwmWrite(base,0);
// if(joystick.axes[2] > 0 ){
//   digitalWrite(baseDir,0);
//   pwmWrite(base, 5);
// }
//
// if(joystick.axes[2] < 0){
//   digitalWrite(baseDir,1);
//   pwmWrite(base, 5);
// }
  
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

void setup()
{
  InitTimersSafe(); 
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(shoulder,155);
  pinMode(shoulderDir, OUTPUT);

  pinMode(elbow,155);
  pinMode(elbowDir, OUTPUT);

//  pinMode(base,155);
//  pinMode(baseDir, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
