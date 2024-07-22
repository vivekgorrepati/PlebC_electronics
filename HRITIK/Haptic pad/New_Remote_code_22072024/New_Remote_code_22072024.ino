const int joystickPinX = A0;  // Analog pin connected to the joystick's X output
const int joystickPinY = A1;  // Analog pin connected to the joystick's Y output
const int joystickPinZ = A2;  // Analog pin connected to the joystick's Z output
const int Potentiometer = A3; // Analog pin connected to the potentiometer

int numberX = 0;
int numberY = 0;
int numberZ = 0;
int velocity = 0;
int prevvelocity = -1; // Initialize to an invalid value

void setup() {
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  int joystickValueX = analogRead(joystickPinX);
  int joystickValueY = analogRead(joystickPinY);
  int joystickValueZ = analogRead(joystickPinZ);
  int PotentiometerValue = analogRead(Potentiometer); 

  // Map potentiometer value to 20 distinct values
  velocity = map(PotentiometerValue, 0, 1023, 1, 20);

  // Print potentiometer mapped value only if it changes
  if (velocity != prevvelocity) {
    Serial.print("V : ");
    Serial.println(velocity);
    prevvelocity = velocity;
  }

  // Handle X-axis
  handleJoystickMovement(joystickValueX, numberX, 'X', velocity);
  // Handle Y-axis
  handleJoystickMovement(joystickValueY, numberY, 'Y', velocity);
  // Handle Z-axis
  handleJoystickMovement(joystickValueZ, numberZ, 'Z', velocity);

  delay(10);  // Common delay to avoid excessive serial prints
}

void handleJoystickMovement(int value, int &number, char axis, int velocity) {
  int prevNumber = number; // Store previous number value

  float Delay = ((1.0 / velocity) * 1000);
  
  // Update number based on joystick value
  if (value > 600 && value <= 1023) {
    number++;
    delay(Delay);
    
    if (number > 800) number = 800;  // Ensure number stays within the range
  } 
  
  else if (value >= 0 && value < 420) {
    number--;
    delay(Delay);
    
    if (number < 0) number = 0;  // Ensure number stays within the range
  } 
  
  // Print only if the number has changed and the joystick is not in neutral position
  if (number != prevNumber && (value < 420 || value > 600)) {
    Serial.print(axis);
    Serial.print(" : ");
    Serial.println(number);
  }
}
