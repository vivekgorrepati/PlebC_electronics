#include <AccelStepper.h>

// // Define Motor pins
// #define MOTOR1_STEP_PIN 10
// #define MOTOR1_DIR_PIN 9

// #define MOTOR2_STEP_PIN 3
// #define MOTOR2_DIR_PIN 48

// #define MOTOR3_STEP_PIN 4
// #define MOTOR3_DIR_PIN 52

// Define Motor pins
#define MOTOR1_STEP_PIN 3
#define MOTOR1_DIR_PIN 4

#define MOTOR2_STEP_PIN 5
#define MOTOR2_DIR_PIN 6

#define MOTOR3_STEP_PIN 7
#define MOTOR3_DIR_PIN 8

// Create instances for the motors
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

void setup() {
  Serial.begin(115200);
  
  motor1.setMaxSpeed(1000);  // Set maximum speed for Motor 1
  motor2.setMaxSpeed(1000);  // Set maximum speed for Motor 2
  motor3.setMaxSpeed(1000);  // Set maximum speed for Motor 3
}

void loop() {
  // Check if there is any serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the input until a newline
    input.trim();  // Clean the input
    processInput(input);  // Process the input string
  }

  // Continuously run the motors
  motor1.runSpeed();
  motor2.runSpeed();
  motor3.runSpeed();
}

// Function to process the input string
void processInput(String input) {
  // Remove the first '$' character if it exists
  if (input[0] == '$') {
    input = input.substring(1);
  }

  // Split the string based on '$' delimiter
  String data[5];  // Array to hold split string parts
  int dataIndex = 0;

  for (int i = 0; i < input.length(); i++) {
    if (input[i] == '$') {
      dataIndex++;
    } else {
      data[dataIndex] += input[i];
    }
  }

  // Process Motor 1 direction (2nd value in the string)
  if (data[1] == "x1") {
    motor1.setSpeed(200);  // Set speed forward for Motor 1
  } else if (data[1] == "-x1") {
    motor1.setSpeed(-200);  // Set speed backward for Motor 1
  } else {
    motor1.setSpeed(0);  // Stop Motor 1 if no valid direction
  }

  // Process Motor 2 direction (3rd value in the string)
  if (data[2] == "y1") {
    motor2.setSpeed(200);  // Set speed forward for Motor 2
  } else if (data[2] == "-y1") {
    motor2.setSpeed(-200);  // Set speed backward for Motor 2
  } else {
    motor2.setSpeed(0);  // Stop Motor 2 if no valid direction
  }

  // Process Motor 3 direction (4th value in the string)
  if (data[3] == "x2") {
    motor3.setSpeed(200);  // Set speed forward for Motor 3
  } else if (data[3] == "-x2") {
    motor3.setSpeed(-200);  // Set speed backward for Motor 3
  } else {
    motor3.setSpeed(0);  // Stop Motor 3 if no valid direction
  }
}
