#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <mcp_can.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define CAN_CS_PIN 8
#define CAN_ID 0x600

MCP_CAN CAN0(CAN_CS_PIN);

const long pulses_per_rev = 2654208;

// Constants
const int elbowPwm = 9;  // Pin 9 is connected to Timer1
const int elbowDir = 7;
const uint8_t pwmfreq = 200;
const uint32_t frequency_elbow = 15000; // 15 kHz PWM frequency

// Timer1 TOP value for desired frequency
const uint16_t timer1Top = F_CPU / frequency_elbow - 1;

// Motor control variables
volatile bool motorRunning = false;
volatile unsigned long motorDuration = 0;
volatile bool motorDirection = true;
const float time_scalar = 2.85;
volatile float elbowCurrentAngle = 0;

// Counter for timing (1ms resolution)
volatile uint16_t ms_counter = 0;


ros::NodeHandle nh;

void runMotor3(uint8_t speed, bool forward, unsigned long duration) {
  motorRunning = true;
  motorDuration = duration;
  motorDirection = forward;
  digitalWrite(elbowDir, forward ? HIGH : LOW);
  setPwmDuty(speed);
  ms_counter = 0;  // Reset millisecond counter
}



void elbowSetup() {
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);

  pinMode(elbowPwm, OUTPUT);
  pinMode(elbowDir, OUTPUT);
  
  // Set up Timer1 for PWM and timing
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Set Fast PWM mode using ICR1 as TOP
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  
  // Set non-inverting mode on OC1A (Pin 9)
  TCCR1A |= (1 << COM1A1);
  
  // Set prescaler to 1
  TCCR1B |= (1 << CS10);
  
  // Set PWM frequency
  ICR1 = timer1Top;
  
  // Initialize PWM duty cycle to 0 (motor stopped)
  OCR1A = 0;
  
  // Enable Timer1 overflow interrupt
  TIMSK1 |= (1 << TOIE1);
  
  digitalWrite(elbowDir, LOW);
  
  // Enable global interrupts
  sei();
  
  Serial.print("PWM frequency set to ");
  Serial.print(frequency_elbow);
  Serial.println(" Hz using Timer1");
  Serial.println("Elbow PWM initialized to 0");
}

void canSetup() {
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN init ok");
  } else {
    Serial.println("CAN init fail");
    while (1);
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("CAN BUS Shield init OK!");

  // Set to Position Mode
  uint8_t modeData1[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
  uint8_t modeData2[8] = {0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
  sendMotorCommand(CAN_ID, modeData1, 8);
  sendMotorCommand(CAN_ID, modeData2, 8);
  
  // Set Speed and Acceleration
  uint8_t speedData[8] = {0x23, 0x81, 0x60, 0x00, 0x54, 0x01, 0x00, 0x00};
  uint8_t accelData[8] = {0x23, 0x83, 0x60, 0x00, 0x10, 0x16, 0x00, 0x00};
  sendMotorCommand(CAN_ID, speedData, 8);
  sendMotorCommand(CAN_ID, accelData, 8);

  // Set speeds and accelerations for other motors
  setSpeedAndAccel(0x601, 0x60, 0x01, 0x10, 0x16);
  setSpeedAndAccel(0x602, 0xF0, 0x00, 0x10, 0x16);
  setSpeedAndAccel(0x604, 0xF0, 0x00, 0x10, 0x16);

  // Set Electronic Gear to Default (8192 pulses per revolution)
  uint8_t electronicGearData[8] = {0x23, 0x90, 0x60, 0x00, 0x00, 0x20, 0x00, 0x00};
  sendMotorCommand(CAN_ID, electronicGearData, 8);
}

void setSpeedAndAccel(uint16_t id, uint8_t speedLow, uint8_t speedHigh, uint8_t accelLow, uint8_t accelHigh) {
  uint8_t speedData[8] = {0x23, 0x81, 0x60, 0x00, speedLow, speedHigh, 0x00, 0x00};
  uint8_t accelData[8] = {0x23, 0x83, 0x60, 0x00, accelLow, accelHigh, 0x00, 0x00};
  sendMotorCommand(id, speedData, 8);
  sendMotorCommand(id, accelData, 8);
}

void setPwmDuty(uint8_t duty) {
  OCR1A = (uint16_t)duty * (timer1Top + 1) / 256;
}

void loop() {

  nh.spinOnce();
  delay(10); // Small delay to control publish rate (100 Hz)

}

void processSerialInput(const std_msgs::String& msg) {
  int id = CAN_ID;
  float angle = 0.0;
  
  String dataCopy = msg.data;  // Make a copy of the string


  
  int spaceIndex =   dataCopy.indexOf(' ');
  if (spaceIndex > 0) {
    String idString =   dataCopy.substring(0, spaceIndex);
    String angleString = dataCopy.substring(spaceIndex + 1);
    

    if (idString.toInt() == 3) {
      processElbowMotor(angleString.toFloat());
      return;
    } 
    
    id += idString.toInt();
    angle = angleString.toFloat();

    Serial.print("ID: ");
    Serial.println(id);
    Serial.print("Angle: ");
    Serial.println(angle);

    nh.logdebug(angle);

    processMotorCommand(id, angle);
  } else {
    Serial.println("Invalid input format. Use 'id angle'.");
  }
}

ros::Subscriber<std_msgs::String> right_pwm_sub("arm_control", &processSerialInput);


void processElbowMotor(float angle) {
  Serial.print("Moving Motor 3 to angle: ");
  Serial.println(angle);
  long angleDiff = angle - elbowCurrentAngle;
  Serial.print("Motor 3 angle diff: ");
  Serial.println(angleDiff);
  elbowCurrentAngle = angle;

  // Calculate duration based on the absolute value of the angle
  long duration = (long)abs(angleDiff) * 6825L / 90L;
  
  // Determine direction based on angle sign
  bool direction = (angleDiff >= 0);
  
  // Run the motor
  runMotor3(pwmfreq, direction, duration);
  
  Serial.print("Moving Motor 3 to angle: ");
  Serial.print(angle);
  Serial.print(", Duration: ");
  Serial.print(duration);
  Serial.print(", Direction: ");
  Serial.println(direction ? "Positive" : "Negative");
}

void processMotorCommand(int id, float angle) {
  // Apply home position offsets
  switch(id) {
    case 0x600: return;
    case 0x601: angle -= 65; break;
    case 0x602: angle -= 71; break;
    case 0x604: angle -= 122; break;
    case 0x605: angle -= 87; break;
    case 0x606: angle += 17; break;
  }

  long targetPosition = (angle / 360.0) * pulses_per_rev;
  if(id == 0x601) {
    targetPosition *= 1.2458;
  }
  if(id == 0x604 || id == 0x605 || id == 0x606) {
    targetPosition *= 0.63;
  }

  // Set Target Position
  uint8_t data[8] = {0x23, 0x7A, 0x60, 0x00};
  memcpy(&data[4], &targetPosition, 4);
  sendMotorCommand(id, data, 8);
  
  // Execute the Position Command and Enable Operation
  uint8_t execData[8] = {0x2B, 0x40, 0x60, 0x00, 0x3F, 0x00, 0x00, 0x00};
  sendMotorCommand(id, execData, 8);
}

void sendMotorCommand(uint16_t id, uint8_t* data, uint8_t dataSize) {
  if (CAN0.sendMsgBuf(id, 0, dataSize, data) == CAN_OK) {
    Serial.print("Sent CAN message: ID=");
    Serial.print(id, HEX);
    Serial.print(", Data=");
    for (size_t i = 0; i < dataSize; i++) {
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Failed to send CAN message");
  }
}

void setup() {
  nh.initNode();
  nh.subscribe(right_pwm_sub);
  
  elbowSetup();

  Serial.println("Elbow setup DONEEEEEEEEEE ...");

  canSetup();
}

// Timer1 Overflow Interrupt Service Routine
ISR(TIMER1_OVF_vect) {
  // This ISR is called at the PWM frequency (15 kHz)
  // We need to count to create a 1ms timer
  static uint16_t pwm_counter = 0;
  pwm_counter++;
  
  if (pwm_counter >= 15) {  // 15 kHz / 15 = 1 kHz (1ms)
    pwm_counter = 0;
    ms_counter++;
    
    if (motorRunning) {
      if (ms_counter >= motorDuration) {
        setPwmDuty(0); // Stop motor
        motorRunning = false;
      }
    }
  }
}
