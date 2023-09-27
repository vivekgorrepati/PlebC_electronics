// Define the digital pins for the encoder phases
const int phaseAPin = 2; // Replace with your actual pin numbers
const int phaseBPin = 3;

// Variables to store encoder values
volatile long encoderValue = 0;
int lastEncoded = 0;
long encoderValueDegrees = 0;

void setup() {
  // Initialize encoder pins as inputs
  pinMode(phaseAPin, INPUT);
  pinMode(phaseBPin, INPUT);

  // Enable pullup resistors for the encoder pins
  digitalWrite(phaseAPin, HIGH);
  digitalWrite(phaseBPin, HIGH);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(phaseAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(phaseBPin), updateEncoder, CHANGE);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read the encoder value in degrees
  encoderValueDegrees = encoderValue * 0.15; // Using the conversion factor

  // Print the encoder value in degrees
  Serial.println(encoderValue);
  Serial.print("Encoder Value (Degrees): ");
  Serial.println(encoderValueDegrees);

  // You can perform other actions based on the encoder value here

  delay(200); // Delay for stability, adjust as needed
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
