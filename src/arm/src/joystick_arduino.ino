
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <PWM.h>

ros::NodeHandle  nh;

int32_t frequency = 10000;
int shoulder = 3;
int shoulderDir = 2;
int elbow = -1;
int elbowDir = -1;

void messageCb( const sensor_msgs::Joy& joystick){
  frequency = abs(map(joystick.axes[1]*100, -100, 100, -30000, 30000));
  SetPinFrequencySafe(3, frequency);
  
  if (joystick.axes[3] > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  pwmWrite(shoulder, 0);
  if (joystick.axes[1] > 0) {
    digitalWrite(shoulderDir, 0);
    pwmWrite(shoulder, 155);
  }
  if (joystick.axes[1] < 0) {
    digitalWrite(shoulderDir, 1);
    pwmWrite(shoulder, 155);
  }
}

ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );

void setup()
{
  InitTimersSafe(); 
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3,155);
  pinMode(2, OUTPUT);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
