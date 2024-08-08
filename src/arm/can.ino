#include <SPI.h>
#include <mcp_can.h>

// Define CAN controller's CS pin
#define CAN_CS_PIN 10

#define CAN_ID 0x600

// Create an MCP_CAN object
MCP_CAN CAN0(CAN_CS_PIN);

const long pulses_per_rev = 2654208; //2 654 208 pulses in one full revolution.

void setup() {
  Serial.begin(115200);

  // Initialize MCP2515 CAN controller at 1 Mbps
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN init ok");
  } else {
    Serial.println("CAN init fail");
    while (1);
  }

  // Set operation mode to normal mode
  CAN0.setMode(MCP_NORMAL);

  Serial.println("CAN BUS Shield init OK!");

  // Set to Position Mode
  uint8_t modeData1[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00}; // Start + voltage output + allow emergency stop + allow operation
  uint8_t modeData2[8] = {0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00}; // Working mode set to position mode
 
  sendMotorCommand(CAN_ID, modeData1, 8);
  sendMotorCommand(CAN_ID, modeData2, 8);
  
  // Set Speed and Acceleration
  uint8_t speedData[8] = {0x23, 0x81, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00}; //speed 1000
 // uint8_t speedData[8] = {0x23, 0x81, 0x60, 0x00, 0xB8, 0x0B, 0x00, 0x00}; // speed 3000
  uint8_t accelData[8] = {0x23, 0x83, 0x60, 0x00, 0x20, 0x4E, 0x00, 0x00};
  sendMotorCommand(CAN_ID, speedData, 8);
  sendMotorCommand(CAN_ID, accelData, 8);

  // Set Electronic Gear to Default (8192 pulses per revolution)
  uint8_t electronicGearData[8] = {0x23, 0x90, 0x60, 0x00, 0x00, 0x20, 0x00, 0x00}; // 8192 = 0x2000 in hexadecimal
  sendMotorCommand(CAN_ID, electronicGearData, 8);
}

void loop() {
  if (Serial.available() > 0) {
    int id = CAN_ID;
    float angle = 0.0;
    String inputString = Serial.readStringUntil('\n'); // Read until newline
    
    // Split the input string into parts using space as delimiter
    int spaceIndex = inputString.indexOf(' ');
    if (spaceIndex > 0) {
        // Extract the ID and angle from the input string
        String idString = inputString.substring(0, spaceIndex);
        String angleString = inputString.substring(spaceIndex + 1);
        
        // Convert the extracted strings to integers and floats
        id += idString.toInt();
        angle = angleString.toFloat();
        
        // Now you can use id and angle as needed
        Serial.print("ID: ");
        Serial.println(id);
        Serial.print("Angle: ");
        Serial.println(angle);
        
        // Assuming CAN_ID is a constant or a previously defined variable
        // You can use id in your CAN_ID calculation if necessary
        // For example:
        // int canId = CAN_ID + id;
    } else {
        Serial.println("Invalid input format. Use 'id angle'.");
    }

    // Convert angle to pulses
    long targetPosition = (angle / 360.0) * pulses_per_rev;
    targetPosition = targetPosition % pulses_per_rev; // Normalize to range between -360 and 360

    // Set Target Position
    uint8_t data[8] = {0x23, 0x7A, 0x60, 0x00};
    memcpy(&data[4], &targetPosition, 4);
    sendMotorCommand(id, data, 8);
    
    // Execute the Position Command and Enable Operation
    uint8_t execData[8] = {0x2B, 0x40, 0x60, 0x00, 0x3F, 0x00, 0x00, 0x00}; // 0x3F: Enable operation
    sendMotorCommand(id, execData, 8);
  }
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