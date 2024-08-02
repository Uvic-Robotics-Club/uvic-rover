#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <Encoder.h>

// Define pins for each motor
const int MOTOR_PINS[4][3] = {
  {2, 3, 4},   // Motor 1: Encoder A, Encoder B, PWM
  {5, 6, 7},   // Motor 2: Encoder A, Encoder B, PWM
  {8, 9, 10},  // Motor 3: Encoder A, Encoder B, PWM
  {11, 12, 13} // Motor 4: Encoder A, Encoder B, PWM
};

const int MOTOR_DIR_PINS[4] = {22, 24, 26, 28}; // Direction pins for each motor

// Create Encoder objects
Encoder encoders[4] = {
  Encoder(MOTOR_PINS[0][0], MOTOR_PINS[0][1]),
  Encoder(MOTOR_PINS[1][0], MOTOR_PINS[1][1]),
  Encoder(MOTOR_PINS[2][0], MOTOR_PINS[2][1]),
  Encoder(MOTOR_PINS[3][0], MOTOR_PINS[3][1])
};

// Constants
const int PPR = 11;
const float GEAR_RATIO = 1.0 / 522.0;
const float DEGREES_PER_TICK = 360.0 / (PPR * 4 / GEAR_RATIO);
const float MAX_ANGLE = 45.0;

// Variables
long homePositions[4] = {0};
long currentPositions[4] = {0};
bool isHomed[4] = {false, false, false, false};

ros::NodeHandle nh;

// void motorCommandCallback(const std_msgs::Float32MultiArray& cmd_msg) {
//   for (int i = 0; i < 4; i++) {
//     if (isHomed[i]) {
//       float targetAngle = constrain(cmd_msg.data[i], -MAX_ANGLE, MAX_ANGLE);
//       moveMotorToAngle(i, targetAngle);
//     }
//   }
// }

void homeCommandCallback(const std_msgs::Bool& home_msg) {
  if (home_msg.data) {
    for (int i = 0; i < 4; i++) {
      homeMotor(i);
    }
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> motor_sub("motor_commands", &motorCommandCallback);
// ros::Subscriber<std_msgs::Bool> home_sub("home_command", &homeCommandCallback);

std_msgs::Float32MultiArray encoder_msg;
ros::Publisher encoder_pub("encoder_feedback", &encoder_msg);

void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i][2], OUTPUT);
    pinMode(MOTOR_DIR_PINS[i], OUTPUT); // asumes it is 0-3 dir pins associated with motor pins possibly not right
  }
  
  nh.initNode();
  nh.subscribe(motor_sub);
  // nh.subscribe(home_sub);
  nh.advertise(encoder_pub);
  float encoder_data[4];

  encoder_msg.data_length = 4;
  encoder_msg.data = encoder_data;
}

void loop() {
  updateEncoders();
  nh.spinOnce();
  delay(10);
}

void updateEncoders() {
  for (int i = 0; i < 4; i++) {
    currentPositions[i] = encoders[i].read();
    float currentAngle = (currentPositions[i] - homePositions[i]) * DEGREES_PER_TICK;
    encoder_msg.data[i] = currentAngle;
  }
  encoder_pub.publish(&encoder_msg);
}

void moveMotor(int motorIndex, int speed, bool forward) {
  digitalWrite(MOTOR_DIR_PINS[motorIndex], forward ? HIGH : LOW);
  analogWrite(MOTOR_PINS[motorIndex][2], speed);
}

void moveMotorToAngle(int motorIndex, float targetAngle) {
  long targetPosition = homePositions[motorIndex] + (long)(targetAngle / DEGREES_PER_TICK);
  
  while (currentPositions[motorIndex] != targetPosition) {
    currentPositions[motorIndex] = encoders[motorIndex].read();
    
    if (currentPositions[motorIndex] < targetPosition) {
      moveMotor(motorIndex, 150, true);  // Move forward
    } else if (currentPositions[motorIndex] > targetPosition) {
      moveMotor(motorIndex, 150, false);  // Move backward
    }
  }
  
  moveMotor(motorIndex, 0, true);  // Stop motor
}

void homeMotor(int motorIndex) {
  // In a real setup, you'd use a limit switch or sensor here
  // For this example, we'll just set the current position as home
  encoders[motorIndex].write(0);
  homePositions[motorIndex] = 0;
  currentPositions[motorIndex] = 0;
  isHomed[motorIndex] = true;
}
