#define xstepPin 9
#define xdirPin 3
//#define xenablePin 2

#define ystepPin 10
#define ydirPin 5
//#define yenablePin 4

#define zstepPin 11
#define zdirPin 6
//#define zenablePin 7

#define xLimitSwL 8
#define xLimitSwR 12

#define yLimitSwL 13
#define yLimitSwR 2

#define zLimitSwL 7
#define zLimitSwR 4

bool xFlag = true;
bool yFlag = true;
bool zFlag = true;

unsigned long previousMillisX = 0;
unsigned long previousMillisY = 0;
unsigned long previousMillisZ = 0;
const long interval = 10;  // Interval to update motors (in milliseconds)

const int stepDelay = 400;  // Delay for stepper motor stepping (in microseconds)
const int YstepDelay = 50;

void setup() {
  Serial.begin(9600);

  // Initialize motor control pins
  pinMode(xstepPin, OUTPUT);
  pinMode(xdirPin, OUTPUT);
  // pinMode(xenablePin, OUTPUT);
  // digitalWrite(xenablePin, LOW);

  pinMode(ystepPin, OUTPUT);
  pinMode(ydirPin, OUTPUT);
  // pinMode(yenablePin, OUTPUT);
  // digitalWrite(yenablePin, LOW);

  pinMode(zstepPin, OUTPUT);
  pinMode(zdirPin, OUTPUT);
  // pinMode(zenablePin, OUTPUT);
  // digitalWrite(zenablePin, LOW);

  // Initialize limit switch pins
  pinMode(xLimitSwL, INPUT_PULLUP);
  pinMode(xLimitSwR, INPUT_PULLUP);
  pinMode(yLimitSwL, INPUT_PULLUP);
  pinMode(yLimitSwR, INPUT_PULLUP);
  pinMode(zLimitSwL, INPUT_PULLUP);
  pinMode(zLimitSwR, INPUT_PULLUP);
}

void loop() {
  unsigned long currentMillis = millis();

  // Task X
  if (currentMillis - previousMillisX >= interval) {
    previousMillisX = currentMillis;
    handleMotorMovement(xLimitSwL, xLimitSwR, xFlag, xmoveMotorForward, xmoveMotorBackward);
  }

  // Task Y
  if (currentMillis - previousMillisY >= 1) {
    previousMillisY = currentMillis;
    handleMotorMovement(yLimitSwL, yLimitSwR, yFlag, ymoveMotorForward, ymoveMotorBackward);
  }

  // Task Z
  if (currentMillis - previousMillisZ >= interval) {
    previousMillisZ = currentMillis;
    handleMotorMovement(zLimitSwL, zLimitSwR, zFlag, zmoveMotorForward, zmoveMotorBackward);
  }
}

void handleMotorMovement(int limitSwL, int limitSwR, bool &flag, void (*moveForward)(), void (*moveBackward)()) {
  if (digitalRead(limitSwL) == LOW) {
    flag = true;
  } else if (digitalRead(limitSwR) == LOW) {
    flag = false;
  }

  if (flag) {
    moveForward();
  } else {
    moveBackward();
  }
}

void xmoveMotorForward() {
  moveMotor(xdirPin, xstepPin, HIGH, stepDelay);
}

void xmoveMotorBackward() {
  moveMotor(xdirPin, xstepPin, LOW, stepDelay);
}

void ymoveMotorForward() {
  moveMotor(ydirPin, ystepPin, HIGH, YstepDelay);
}

void ymoveMotorBackward() {
  moveMotor(ydirPin, ystepPin, LOW, YstepDelay);
}

void zmoveMotorForward() {
  moveMotor(zdirPin, zstepPin, HIGH, stepDelay);
}

void zmoveMotorBackward() {
  moveMotor(zdirPin, zstepPin, LOW, stepDelay);
}

void moveMotor(int dirPin, int stepPin, bool dirState, int delayTime) {
  digitalWrite(dirPin, dirState);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(delayTime);
}