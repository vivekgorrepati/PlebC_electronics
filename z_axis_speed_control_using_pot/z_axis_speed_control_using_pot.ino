unsigned long prevTime = millis(); // Variable to store the previous time

#include <AccelStepper.h>

int upState = 0;
int dnState = 0;

//int upperstate = 1;
//int lowerstate = 1;

const int joyXPin = A0; // Analog input pin for joystick X-axis
const int potPin = A3;  // Analog input pin for the potentiometer

float a = 1000; // Initial acceleration value

#define STEP_PIN 4
#define DIR_PIN 52
#define upper_limit_pin 5
#define lower_limit_pin 6

AccelStepper yaxis(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  yaxis.setMaxSpeed(5000.0); // Set the maximum speed of the stepper motor
  pinMode(upper_limit_pin, INPUT_PULLUP);
  pinMode(lower_limit_pin, INPUT_PULLUP);
}

void loop() {
  unsigned long currentTime = millis(); // Get the current time
  if (currentTime - prevTime > 0) {
    float xValue = analogRead(joyXPin); // Read the X-axis value from the joystick
    int upperstate = digitalRead(upper_limit_pin);
    int lowerstate = digitalRead(lower_limit_pin);

    // Read the analog value from the potentiometer
    int potValue = analogRead(potPin);
    // Map the potentiometer value to the desired acceleration range
    a = map(potValue, 0, 1023, 100, 500); // Adjust the range as needed

    if ((xValue > 1000) && (upperstate == 1)) { // If the X-axis value is greater than 800
      yaxis.setSpeed(a); // Set the speed of the stepper motor to a positive value
      yaxis.runSpeed(); // Run the stepper motor at the set speed
    } else if ((xValue < 100) && (lowerstate == 1)) { // If the X-axis value is less than 200
      yaxis.setSpeed(-(a)); // Set the speed of the stepper motor to a negative value
      yaxis.runSpeed(); // Run the stepper motor at the set speed
    } else {
      yaxis.stop(); // Stop the stepper motor if the X-axis value is between 200 and 800
    }

    prevTime = currentTime; // Update the previous time
  }
}