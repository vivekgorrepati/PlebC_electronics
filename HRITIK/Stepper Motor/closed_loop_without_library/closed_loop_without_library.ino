// Define the stepper motor connections
#define stepPin 6
#define dirPin 7
#define enablePin 8

int encoderPin1 = 2;
int encoderPin2 = 3;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long int currentPosition = 0;
float targetPosition = 0;
long int positionToMove = 0;
int stepsToMove = 0, angleStore = 0;
const int motorSetSteps = 6400; // Define these as constants
const int encoderResolution = 4000;

int prevstepsToMove = 0;
int prev_angleStore = 0;

void setup() {
  Serial.begin(115200);

  // Set up stepper motor control pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // Disable the motor
  
  // Set up encoder pins
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); // turn pullup resistor on

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    String angleReceived = Serial.readStringUntil('\n'); // Read the angle from the serial monitor
    angleReceived.trim();
    int intAngleReceived = angleReceived.toInt();
    if (intAngleReceived >= 0 && intAngleReceived <= 360) {
      angleStore = intAngleReceived;
      Serial.println(angleStore);
    }
  }

  currentPosition = encoderValue;
  targetPosition = angleStore * ((float)encoderResolution / 360.0);
  positionToMove = (long int)targetPosition - currentPosition;
  stepsToMove = positionToMove * (motorSetSteps / (float)encoderResolution);

  if ((prev_angleStore != angleStore) || (prevstepsToMove != stepsToMove))
	{
	//Enable Drive
	digitalWrite(enablePin, LOW); // Disable the motor
	// Move motor
		moveMotor(stepsToMove);

	}
  if ((prev_angleStore == angleStore) || (prevstepsToMove == stepsToMove))
	{
		//Disable Drive
		digitalWrite(enablePin, HIGH); // Disable the motor
	}

  prev_angleStore = angleStore;
	prevstepsToMove = stepsToMove;

  delay(50);
}

void moveMotor(int steps) {
  if (steps < 0) {
    digitalWrite(dirPin, LOW); // Set direction
    steps = -steps; // Make steps positive
  } else {
    digitalWrite(dirPin, HIGH);
  }

  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200); // Adjust delay for your motor's speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
  }
}

void updateEncoder() {
  int MSB = digitalRead(encoderPin1); // MSB = most significant bit
  int LSB = digitalRead(encoderPin2); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; // store this value for next time
}
