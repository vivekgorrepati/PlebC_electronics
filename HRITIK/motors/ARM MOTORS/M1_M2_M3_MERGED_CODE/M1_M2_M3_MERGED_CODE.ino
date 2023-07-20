//motor 1
#include <AccelStepper.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution = 400;

//motor 1
float previousValue_1 = 0;
float resultValue_1 = 0;
bool isRotating_1 = false;

//motor 2
float previousValue_2 = 0;
float resultValue_2 = 0;
bool isRotating_2 = false;

//motor 3
float previousValue_3 = 0;
float resultValue_3 = 0;
bool isRotating_3 = false;

// Gear ratio for motor 1
const float gearRatio_1 = 2.0; // Gear ratio of 13:1 (output shaft:input shaft)
// Gear ratio for motor 2
const float gearRatio_2 = 13.0; // Gear ratio of 13:1 (output shaft:input shaft)

// Create an instance of the AccelStepper class - motor 1
AccelStepper stepper1(AccelStepper::DRIVER, 11, 8);  // Assuming the stepper motor is connected to pins 11 (STEP_PIN) and 8 (DIR_PIN)

// Create an instance of the AccelStepper class - motor 1
AccelStepper stepper2(AccelStepper::DRIVER, 10, 7);

// Create an instance of the AccelStepper class - motor 1
AccelStepper stepper3(AccelStepper::DRIVER, 9, 4);  // Assuming the stepper motor is connected to pins 11 (STEP_PIN) and 8 (DIR_PIN)

unsigned long prevTime = millis();

// Function prototypes
void degree_recieve_m1();
void degree_recieve_m2();
void degree_recieve_m3();
void motor_1(float degrees_1);
void motor_2(float degrees_2);
void motor_3(float degrees_3);


void setup() {
  Serial.begin(9600);

  //motor 1
  stepper1.setMaxSpeed(1000);     // Set the maximum speed in steps per second
  stepper1.setAcceleration(1000); // Set the acceleration in steps per second per second
  stepper1.setCurrentPosition(0); // Set the current position to 0

  //motor 2
  stepper2.setMaxSpeed(1000);     // Set the maximum speed in steps per second
  stepper2.setAcceleration(1000); // Set the acceleration in steps per second per second
  stepper2.setCurrentPosition(0);

  //motor 3
  stepper3.setMaxSpeed(1000);     // Set the maximum speed in steps per second
  stepper3.setAcceleration(1000); // Set the acceleration in steps per second per second
  stepper3.setCurrentPosition(0); // Set the current position to 0

  millis();

}
float m1angle =0;
float m2angle =0;
float m3angle =0;

void loop() {

 unsigned long currentTime = millis();
   if (currentTime - prevTime  >0)
   {
     if(Serial.available()>0){
      String receivedString = Serial.readStringUntil('\n');

    // Split the string into three float values
      char* token;
      token = strtok(const_cast<char*>(receivedString.c_str()), "$");

      m1angle = atof(token);
      token = strtok(NULL, "$");
      m2angle = atof(token);
      token = strtok(NULL, "$");
      m3angle= atof(token);
     }
     else{
       isRotating_1=false;
       isRotating_2=false;
       isRotating_3=false;
     }
       degree_recieve_m1();
       degree_recieve_m2();
       degree_recieve_m3();
       prevTime = currentTime;
   }
}

void degree_recieve_m1(){
  //if (Serial.available()) {
    
    resultValue_1 = m1angle *2;  // Limit the angle within the range 0-360

    if (resultValue_1 != previousValue_1 && !isRotating_1) {
      motor_1(resultValue_1);
      previousValue_1 = resultValue_1;
      isRotating_1 = true;
    }
    
  //} 
  //else {
  //  isRotating_1 = false; // Reset the isRotating flag if no input is available
  //}
}

void degree_recieve_m2(){
  //if (Serial.available()) {
    //int degrees_2 = Serial.parseInt();
    resultValue_2 = m2angle * 13;  // Limit the angle within the range 0-360

    if (resultValue_2 != previousValue_2 && !isRotating_2) {
      motor_2(resultValue_2);
      previousValue_2 = resultValue_2;
      isRotating_2 = true;
    }
    
  //} else {
  //  isRotating_2 = false; // Reset the isRotating flag if no input is available
  //}
}

void degree_recieve_m3(){
  //if (Serial.available()) {
    //int degrees_3 = Serial.parseInt();
    resultValue_3 = m3angle; //% 360;  // Limit the angle within the range 0-360

    if (resultValue_3 != previousValue_3 && !isRotating_3) {
      motor_3(resultValue_3);
      previousValue_3 = resultValue_3;
      isRotating_3 = true;
    }
    
  //} else {
  //  isRotating_3 = false; // Reset the isRotating_3 flag if no input is available
  //}
}

void motor_1(float degrees_1) {
  // Calculate the difference between the previous value and the current value
  float angleDifference_1 = degrees_1 - previousValue_1;
  float rotationDirection_1 = (angleDifference_1 >= 0) ? 1 : -1;  // Determine the rotation direction

  // Calculate the adjusted angle difference considering the gear ratio
  float adjustedangleDifference_1 = angleDifference_1 * gearRatio_1;

  // Convert the adjusted angle difference to steps
  long steps_1 = map(abs(adjustedangleDifference_1), 0, 360* gearRatio_1, 0, stepsPerRevolution) * rotationDirection_1;

  // Get the target position by adding the steps to the current position
  long targetPosition_1 = stepper1.currentPosition() + steps_1;

  // Move the stepper motor to the target position
  stepper1.moveTo(targetPosition_1);

  // Run the stepper motor until it reaches the target position
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  isRotating_1 = false; // Reset the isRotating flag after the rotation is complete
  Serial.println("motor_1");
}

void motor_2(float degrees_2) {
  // Calculate the difference between the previous value and the current value
  float angleDifference_2 = degrees_2 - previousValue_2;
  float rotationDirection_2 = (angleDifference_2 >= 0) ? 1 : -1;  // Determine the rotation direction

  // Calculate the adjusted angle difference considering the gear ratio
  float adjustedAngleDifference_2 = angleDifference_2 * gearRatio_2;

  // Convert the adjusted angle difference to steps
  long steps_2 = map(abs(adjustedAngleDifference_2), 0, 360 * gearRatio_2, 0, stepsPerRevolution) * rotationDirection_2;

  // Get the target position by adding the steps to the current position
  long targetPosition_2 = stepper2.currentPosition() + steps_2;

  // Move the stepper motor to the target position
  stepper2.moveTo(targetPosition_2);

  // Run the stepper motor until it reaches the target position
  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }

  isRotating_2 = false; // Reset the isRotating flag after the rotation is complete
  Serial.println("motor_2");
}

void motor_3(float degrees_3) {
  // Calculate the difference between the previous value and the current value
  float angleDifference_3 = degrees_3 - previousValue_3;
  float rotationDirection_3 = (angleDifference_3 >= 0) ? 1 : -1;  // Determine the rotation direction

  // Convert the angle difference to steps
  long steps_3 = map(abs(angleDifference_3), 0, 360, 0, stepsPerRevolution) * rotationDirection_3;

  // Get the target position by adding the steps to the current position
  long targetPosition_3 = stepper3.currentPosition() + steps_3;

  // Move the stepper motor to the target position
  stepper3.moveTo(targetPosition_3);

  // Run the stepper motor until it reaches the target position
  while (stepper3.distanceToGo() != 0) {
    stepper3.run();
  }

  isRotating_3 = false; // Reset the isRotating_3 flag after the rotation is complete
  Serial.println("motor_3");
}
