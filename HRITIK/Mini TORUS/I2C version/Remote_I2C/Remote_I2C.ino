#include <Wire.h>
#define SLAVE_ADDRESS 0x12 // Replace with unique address for each Nano

const int j1XPin = A0;
const int j1YPin = A1;
const int j2XPin = A2;
const int j2YPin = A3;
const int potPin = A6;

String gantryValue; // Global variable for gantryValue

void setup() {
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS); // Initialize I2C communication as slave
  Wire.onRequest(sendData); // Define function to call when master requests data
}

void loop() {
  int potValue = analogRead(potPin);
  int x1Value = analogRead(j1XPin);
  int y1Value = analogRead(j1YPin);
  int x2Value = analogRead(j2XPin);
  int y2Value = analogRead(j2YPin);

  String output = "";

  // Add potValue to the output
  output += "$" + String(potValue);

  // Check and print x1 condition
  if (x1Value > 900) {
    output += "$x1";
  } else if (x1Value < 200) {
    output += "$-x1";
  } else {
    output += "$0";
  }

  // Check and print y1 condition
  if (y1Value > 900) {
    output += "$y1";
  } else if (y1Value < 200) {
    output += "$-y1";
  } else {
    output += "$0";
  }

  // Check and print x2 condition
  if (x2Value > 900) {
    output += "$x2";
  } else if (x2Value < 200) {
    output += "$-x2";
  } else {
    output += "$0";
  }

  // Check and print y2 condition
  if (y2Value > 900) {
    output += "$y2";
  } else if (y2Value < 200) {
    output += "$-y2";
  } else {
    output += "$0";
  }
  gantryValue = "#" + String(output) + "*";
  Serial.println(gantryValue); // Print the formatted output
  
}

void sendData() {
  Wire.write(gantryValue.c_str()); // Convert to C-string and specify length
}




//output
// gantry_value : $1023$0$-y1$0$0
// gantry_value : $1023$0$-y1$0$-y2
// gantry_value : $1023$-x1$0$0$0
// gantry_value : $1023$x1$y1$0$0
// gantry_value : $1023$0$-y1$0$0
// gantry_value : $1023$-x1$0$-x2$0

