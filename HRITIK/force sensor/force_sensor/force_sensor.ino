const int forcePin = A0; // Analog input pin for force sensor
const float voltageRef = 5.0; // Reference voltage of Arduino (5V)

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  int sensorValue = analogRead(forcePin); // Read the analog value from the force sensor
  float voltage = sensorValue * voltageRef / 1023.0; // Convert the sensor value to voltage

  // Assuming you have calibration data for your specific force sensor,
  // you can calculate the pressure in Newtons using a formula.
  // Replace the calibration values with your own.
  float calibrationFactor = 10.0; // Calibration factor for your specific sensor
  float pressure = voltage * calibrationFactor;
  
	// Serial.print("Analog value: ");
  // Serial.println(sensorValue);
  // Serial.print("Voltage value: ");
  // Serial.print(voltage);
  // Serial.println("v");
  // Serial.print("Force (N): ");
  // Serial.println(pressure);

  Serial.print("Analog value: ");
  Serial.print(sensorValue);
  Serial.print('\t');
  Serial.print("Voltage value: ");
  Serial.print(voltage);
  Serial.println("v");

  delay(500); // Delay for stability or adjust as needed
}
