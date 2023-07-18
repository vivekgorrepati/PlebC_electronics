#include <AccelStepper.h>
 int forwardState = 0;
int backwardState = 0;

//int upperstate = 1;
//int lowerstate = 1;

const int joyZPin = A2; // Analog input pin for joystick X-axis

float z = 1000; // Acceleration value

#define zSTEP_PIN 10
#define zDIR_PIN 9
#define forward_limit_pin 5
#define backward_limit_pin 6

AccelStepper zaxis(AccelStepper::DRIVER, zSTEP_PIN, zDIR_PIN);
  void zaxissetup(){
  
  zaxis.setMaxSpeed(5000.0); // Set the maximum speed of the stepper motor
  millis(); // Not sure why this line is here, it doesn't have any effect
  pinMode(forward_limit_pin, INPUT_PULLUP);
  pinMode(backward_limit_pin, INPUT_PULLUP);
    }
  void zaxisloop ()
  {
     
    float zValue = analogRead(joyZPin); // Read the X-axis value from the joystick
   int   forwardstate = digitalRead(forward_limit_pin);
   int   backwardstate = digitalRead(backward_limit_pin);
    if ((zValue > 1000)&&(forwardstate==1)) { // If the X-axis value is greater than 800
      zaxis.setSpeed(z); // Set the speed of the stepper motor to a positive value
      zaxis.runSpeed(); // Run the stepper motor at the set speed
    } 
else 
    if ((zValue < 100)&&(backwardstate==1)) { // If the X-axis value is less than 200
      zaxis.setSpeed(-(z)); // Set the speed of the stepper motor to a negative value
      zaxis.runSpeed(); // Run the stepper motor at the set speed
    } 
else {
      zaxis.stop(); // Stop the stepper motor if the X-axis value is between 200 and 800
      
    }
  }
