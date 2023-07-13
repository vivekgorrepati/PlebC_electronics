#include <AccelStepper.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution = 400;
int previousValue = 0;
int resultValue = 0;
bool isRotating = false;

// Gear ratio
const float gearRatio = 13.0; // Gear ratio of 13:1 (output shaft:input shaft)

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, 11, 8);  // Assuming the stepper motor is connected to pins 11 (STEP_PIN) and 8 (DIR_PIN)

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);     // Set the maximum speed in steps per second
  stepper.setAcceleration(1000); // Set the acceleration in steps per second per second
  stepper.setCurrentPosition(0); // Set the current position to 0
}

void loop() {
  if (Serial.available()) {
    int degrees = Serial.parseInt();
    resultValue = degrees % 13;  // Limit the angle within the range 0-360

    if (resultValue != previousValue && !isRotating) {
      motor(resultValue);
      previousValue = resultValue;
      isRotating = true;
    }
  } else {
    isRotating = false; // Reset the isRotating flag if no input is available
  }
}

void motor(int degrees) {
  // Calculate the difference between the previous value and the current value
  int angleDifference = degrees - previousValue;
  int rotationDirection = (angleDifference >= 0) ? 1 : -1;  // Determine the rotation direction

  // Calculate the adjusted angle difference considering the gear ratio
  int adjustedAngleDifference = angleDifference * gearRatio;

  // Convert the adjusted angle difference to steps
  long steps = map(abs(adjustedAngleDifference), 0, 360 * gearRatio, 0, stepsPerRevolution) * rotationDirection;

  // Get the target position by adding the steps to the current position
  long targetPosition = stepper.currentPosition() + steps;

  // Move the stepper motor to the target position
  stepper.moveTo(targetPosition);

  // Run the stepper motor until it reaches the target position
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  isRotating = false; // Reset the isRotating flag after the rotation is complete
}
