#include <Wire.h>

const int slaveAddress = 0x11;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Start I2C as master
}
int serial = 1;
void loop() {
  String serValue = String(serial) + " : ";
  Serial.print(serValue);
  requestEncoderValue();
  serial++;
  delay(10); // Adjust based on how frequently you want to request the data
}

void requestEncoderValue() {
  Wire.requestFrom(slaveAddress, sizeof(long)); // Request sizeof(long) bytes from the slave
  while (Wire.available() < sizeof(long)) {
    // Wait for the data to be available
  }
  
  long encoderValueReceived = 0;
  Wire.readBytes((char*)&encoderValueReceived, sizeof(encoderValueReceived)); // Read the incoming bytes into encoderValueReceived

  Serial.print("Encoder Value: ");
  Serial.println(encoderValueReceived);
}
