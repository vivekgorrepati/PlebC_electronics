#include <AccelStepper.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution = 400;

// Gear ratio for each motor
const float gearRatio_1 = 2.0; // Gear ratio for motor 1
const float gearRatio_2 = 13.7; // Gear ratio for motor 2
const float gearRatio_3 = 1.0; // Gear ratio for motor 3

// Create instances of the AccelStepper class for each motor
AccelStepper stepper1(AccelStepper::DRIVER, 5, 8); // Motor 1 - STEP_PIN: 5, DIR_PIN: 8
AccelStepper stepper2(AccelStepper::DRIVER, 10, 7); // Motor 2 - STEP_PIN: 10, DIR_PIN: 7
AccelStepper stepper3(AccelStepper::DRIVER, 9, 4);  // Motor 3 - STEP_PIN: 9, DIR_PIN: 4

// Variables to store the angles for each motor
int angle_1 = 0;
int angle_2 = 0;
int angle_3 = 0;

// Define the digital pins for the encoder phases
const int phaseAPin = 2; // Replace with your actual pin numbers
const int phaseBPin = 3;

// Variables to store encoder values
volatile long encoderValue = 0;
int lastEncoded = 0;
long encoderValueDegrees = 0;

// Define the enable pin for motor 1
const int enablePin1 = 6; // Choose the appropriate pin for motor 1

void setup() {
  Serial.begin(9600);

  // Initialize encoder pins as inputs
  pinMode(phaseAPin, INPUT);
  pinMode(phaseBPin, INPUT);

  // Enable pullup resistors for the encoder pins
  digitalWrite(phaseAPin, HIGH);
  digitalWrite(phaseBPin, HIGH);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(phaseAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseBPin), updateEncoder, CHANGE);

  // Set up each stepper motor
  stepper1.setMaxSpeed(15000); // Set maximum speed in steps per second
  stepper1.setAcceleration(400); // Set acceleration in steps per second per second
  pinMode(enablePin1, OUTPUT);
  digitalWrite(enablePin1, HIGH); // Enable motor 1 by default

  stepper2.setMaxSpeed(40000);
  stepper2.setAcceleration(2000);

  stepper3.setMaxSpeed(5000);
  stepper3.setAcceleration(300);
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

  // Read the encoder value in degrees
  encoderValueDegrees = encoderValue * 0.075; // Using the conversion factor

  // Print the encoder value in degrees
  // Serial.print("Encoder Value (Degrees): ");
  Serial.println(encoderValueDegrees);

  // Compare the encoder feedback angle to the input angle
  

  
  // Control the enable pin for motor 1 
  if (stepper1.run() ) {
    digitalWrite(enablePin1, LOW); // Disable motor 1
  } else {
    digitalWrite(enablePin1, HIGH); // Enable motor 1
  }

  // delay(200); // Delay for stability, adjust as needed
}

void updateEncoder() {
  int MSB = digitalRead(phaseAPin); // Read phase A
  int LSB = digitalRead(phaseBPin); // Read phase B

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderValue++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderValue--;
  }

  lastEncoded = encoded;
}
