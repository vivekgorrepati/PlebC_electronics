#include <Wire.h>

#define SLAVE_ADDRESS 0x11 // Replace with unique address for each Nano

void setup() {
  Wire.begin(SLAVE_ADDRESS); // Initialize I2C communication as slave
  Wire.onRequest(sendData1); // Define function to call when master requests data
}

void loop() {
  // Your main code here
  sendData1();

void sendData1() {
  // Send data to master when requested
  Wire.write("#200$120$150*");
  
}

