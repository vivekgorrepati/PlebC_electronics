#include <Wire.h>

#define SLAVE_ADDRESS 0x11

volatile long encoderValue = 0; // Updated by the ISR
volatile bool valueChanged = false; // Flag set by the ISR when the value changes

int encoderPin1 = 2;
int encoderPin2 = 3;
volatile int lastEncoded = 0;

void setup() {
  Serial.begin(115200);

  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);

  Wire.begin(SLAVE_ADDRESS); // Initialize I2C communication as slave
  Wire.setClock(400000); // Optional: Increase I2C clock speed to 400kHz for faster data transmission
  Wire.onRequest(sendData1); // Register sendData1() to run when data is requested by the master
}

void loop() {
  // Loop does not need to actively do anything
  Serial.println(encoderValue);
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1); // MSB = most significant bit
  int LSB = digitalRead(encoderPin2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to a single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; // store this value for next time
  valueChanged = true; // Set the flag to indicate the value has changed
}

void sendData1() {
  Wire.write((const uint8_t*)&encoderValue, sizeof(encoderValue)); // Send the encoder value as binary data
}
