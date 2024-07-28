// Initial test to attempt to read encoder data

#define ENCODER_OPTIMIZE_INTERRUPTS // Encoder will directly define interrupts the minimum overhead, not compatible if program uses attachInterrupt()
#include <Encoder.h>

Encoder myEnc(2, 3);  // Encoder connected to pins 2 and 3

const int PPR = 11;   // Pulses per revolution
const float GEAR_RATIO = (1.0/522.0);  // Replace with actual gear ratio if different

long oldPosition  = -999;
long newPosition;
unsigned long lastTime = 0;
const unsigned long PUBLISH_INTERVAL = 50;  // Send data every 50ms

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
}

void loop() {
  newPosition = myEnc.read();

  
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= PUBLISH_INTERVAL) {
    // Calculate position
    int32_t position = (newPosition / (4 * PPR * GEAR_RATIO)) * 360;  // Position in degrees
    // Note: Issue may be here, need to look into what .read() does

    // Calculate velocity
    float velocity = (float)(newPosition - oldPosition) / (currentTime - lastTime);  // Pulses per second
    velocity = (velocity / (4 * PPR * GEAR_RATIO)) * 60.0;  // Convert to RPM

    // Send data over serial
    if(oldPosition != newPosition) {
      float printPos = position % 360; 
      Serial.print(printPos);
      Serial.print(",");
      Serial.println(velocity);
    }

    oldPosition = newPosition;
    lastTime = currentTime;
  }
}
