void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
}

void loop() {
  const int numReadings = 10;  // Number of readings to average
  int totalReadings = 0;
  float R1 = 66900;
  float R2 = 9950;

  for (int i = 0; i < numReadings; i++) {
    // Read the voltage from analog pin A1
    int sensorValue = analogRead(A1);
    totalReadings += sensorValue;
    delay(10); // Delay between readings
  }

  // Calculate the average reading
  int averageReading = totalReadings / numReadings;

  // Convert the ADC reading to voltage (assuming 5V reference voltage)
  float voltage = (averageReading * 5.0) / 1023.0;

  // Calculate the actual voltage across the 10k resistor
  // The voltage across the 68k resistor will be 5V - voltage
  float voltageAcross10k = voltage;

  // Calculate the voltage at the junction (between 68k and 10k)
  // It's the voltage across the 10k resistor divided by the voltage divider ratio
  float TotalCurrent = voltageAcross10k / (R2);
  float TotalVoltage = (R1+R2) * TotalCurrent;

  // Print the voltage value at the junction to the Serial Monitor
  Serial.print("Total Voltage: ");
  Serial.print(TotalVoltage, 2); // Display with 2 decimal places
  Serial.println(" V");

  // Wait for a short period before reading again
  delay(1000);
}
