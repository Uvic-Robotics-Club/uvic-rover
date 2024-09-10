#include <ros.h>
#include <Encoder.h>
#include <std_msgs/Int64.h>

// Encoder connected to pins 2 and 3
// FR 2 3 
// FL 4 5

Encoder FREnc(2, 3);  
Encoder FLEnc(4, 5) 
ros::Publisher encoderValue("encoderValue", &encoder);
ros::Subscriber<std_msgs::Int16> pwm("PWM_Values", &pwm_input);


void setup() {


  nh.initNode();
  nh.advertise(encoderValue);
  nh.subscribe(pwm);
  

}


void loop() {
    
  nh.loginfo("Encoder Value");
  
  encoder.data = currentPosition;
  encoderValue.publish( &encoder );
  
  nh.spinOnce();
  delay(100);
  
}


void pwm_input( const std_msgs::Int16& pwm_value){
  int pwm =0;
  pwm = pwm_value.data;
  
  if ( pwm > 0 )
  {
  motorGo(MOTOR_1,CCW,pwm);
  }
  else
  {
  motorGo(MOTOR_1,CW,abs(pwm));
  }
 
}
