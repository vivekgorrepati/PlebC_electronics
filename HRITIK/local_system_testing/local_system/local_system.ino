#include <AccelStepper.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution = 400;

// Gear ratio for each motor
const float gearRatio_1 = 2.0; // Gear ratio for motor 1
const float gearRatio_2 = 13.7; // Gear ratio for motor 2
const float gearRatio_3 = 1.0; // Gear ratio for motor 3

// Create instances of the AccelStepper class for each motor
AccelStepper stepper1(AccelStepper::DRIVER, PA0, PA2); // Motor 1 - STEP_PIN: 11, DIR_PIN: 8
AccelStepper stepper2(AccelStepper::DRIVER, PA1, PA3); // Motor 2 - STEP_PIN: 10, DIR_PIN: 7
AccelStepper stepper3(AccelStepper::DRIVER, PA4, PA5); // Motor 3 - STEP_PIN: 9, DIR_PIN: 4

// Variables to store the angles for each motor
int angle_1 = 0;
int angle_2 = 0;
int angle_3 = 0;

void setup() {
  Serial.begin(115200);

  // Set up each stepper motor
  stepper1.setMaxSpeed(7.299); // Set maximum speed in steps per second
  stepper1.setAcceleration(145.95); // Set acceleration in steps per second per second 600

  stepper2.setMaxSpeed(50);
  stepper2.setAcceleration(1000);  //3000

  stepper3.setMaxSpeed(3); //7.299
  stepper3.setAcceleration(50); //145.95
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