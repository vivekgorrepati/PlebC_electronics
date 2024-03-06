
#include <AccelStepper.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution =2000;

// Gear ratio for each motor
const float gearRatio_1 = 1.0; // Gear ratio for motor 1
const float gearRatio_2 = 1.0; // Gear ratio for motor 2
const float gearRatio_3 = 1.0; // Gear ratio for motor 3

// Create instances of the AccelStepper class for each motor
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepper1(AccelStepper::FULL4WIRE, 6, 8, 7, 9); // Motor 1 - Wire connections
AccelStepper stepper2(AccelStepper::FULL4WIRE, 2, 4, 3, 5); // Motor 2 - Wire connections
AccelStepper stepper3(AccelStepper::FULL4WIRE, 10, 12, 11, 13); // Motor 3 - Wire connections

// Variables to store the angles for each motor
float angle_1 = 0;
float angle_2 = 0;
float angle_3 = 0;

void setup() {
  Serial.begin(9600);

  // Set up each stepper motor
  stepper1.setMaxSpeed(6000); // Set maximum speed in steps per second
  stepper1.setAcceleration(600); // Set acceleration in steps per second per second

  stepper2.setMaxSpeed(6000);
  stepper2.setAcceleration(600);

  stepper3.setMaxSpeed(6000);
  stepper3.setAcceleration(600);
}

void loop() {
  if (Serial.available()) {
    String receivedString = Serial.readStringUntil('\n');

    // Split the received string into angle values
    char* token;
    char* angleStr;
    char receivedChars[receivedString.length() + 1];
    strcpy(receivedChars, receivedString.c_str());

    // Process the first angle separately
    token = strtok(receivedChars, "$");
    angleStr = token;
    angle_1 = atof(angleStr);

    // Process the remaining angles
    token = strtok(NULL, "$");
    int angleIndex = 2;
    while (token != NULL) {
      angleStr = token;
      float angle = atof(angleStr);

      // Assign angles to the corresponding motors
      switch (angleIndex) {
        case 2:
          angle_2 = angle;
          break;
        case 3:
          angle_3 = angle;
          break;
        // Add more cases if needed for additional motors
      }

      token = strtok(NULL, "$");
      angleIndex++;
    }

    // Move each motor to the corresponding angle
    long targetPosition_1 = map(angle_1, 0, 360, 0, stepsPerRevolution) * gearRatio_1;
    long targetPosition_2 = map(angle_2, 0, 360, 0, stepsPerRevolution) * gearRatio_2;
    long targetPosition_3 = map(angle_3, 0, 360, 0, stepsPerRevolution) * gearRatio_3;

    stepper1.moveTo(targetPosition_1);
    stepper2.moveTo(targetPosition_2);
    stepper3.moveTo(targetPosition_3);
  }

  // Update all motors
  stepper1.run();
  stepper2.run();
  stepper3.run();

  // Check if each motor has reached its target position
  if (!stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning()) {
    // All motors have reached their target positions
    // Do something if needed after all motors have stopped moving
  }
}