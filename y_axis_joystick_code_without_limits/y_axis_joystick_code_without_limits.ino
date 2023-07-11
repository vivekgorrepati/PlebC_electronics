unsigned long prevTime = millis(); // Variable to store the previous time

#include <AccelStepper.h>

int upState = 0;
int dnState = 0;

const int joyXPin = A0; // Analog input pin for joystick X-axis

float a = 1000; // Acceleration value

#define STEP_PIN 11
#define DIR_PIN 8

AccelStepper yaxis(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  yaxis.setMaxSpeed(5000.0); // Set the maximum speed of the stepper motor
  millis(); // Not sure why this line is here, it doesn't have any effect
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time
  if (currentTime - prevTime > 0) {
    float xValue = analogRead(joyXPin); // Read the X-axis value from the joystick
    
    if (xValue > 800) { // If the X-axis value is greater than 800
      yaxis.setSpeed(a); // Set the speed of the stepper motor to a positive value
      yaxis.runSpeed(); // Run the stepper motor at the set speed
    } else if (xValue < 200) { // If the X-axis value is less than 200
      yaxis.setSpeed(-(a)); // Set the speed of the stepper motor to a negative value
      yaxis.runSpeed(); // Run the stepper motor at the set speed
    } else {
      yaxis.stop(); // Stop the stepper motor if the X-axis value is between 200 and 800
    }
  }
}
