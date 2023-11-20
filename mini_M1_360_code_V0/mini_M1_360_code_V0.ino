#include <Stepper.h>

// Change these constants according to your motor's specifications
const int stepsPerRevolution = 100;  // change this to fit the number of steps per revolution
const int motorPin1 = 2;
const int motorPin2 = 3;
const int motorPin3 = 4;
const int motorPin4 = 5;

Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

void setup() {
  // Set the speed of the motor
  myStepper.setSpeed(200);  // Set to your desired speed in RPM
  Serial.begin(9600);
}

void loop() {
  Serial.println("Enter the angle (0 to 360): ");
  
  while (Serial.available() == 0);

 int givenAngle = Serial.parseInt();

  int targetAngle = givenAngle*4;

  // Convert degrees to steps
  int stepsToMove = map(targetAngle, 0, 360, 0, stepsPerRevolution * 5.12); 

  // Move the stepper motor
  myStepper.step(stepsToMove);

  delay(100);  // Add a delay for stability
}