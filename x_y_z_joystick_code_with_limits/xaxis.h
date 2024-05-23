// Get the current time
#include <AccelStepper.h>
 int rightState = 0;
int leftState = 0;

//int upperstate = 1;
//int lowerstate = 1;

const int joyXPin = A0; // Analog input pin for joystick X-axis

float x = 1000; // Acceleration value

#define xSTEP_PIN 10
#define xDIR_PIN 9
#define right_limit_pin 5
#define left_limit_pin 6

AccelStepper xaxis(AccelStepper::DRIVER, xSTEP_PIN, xDIR_PIN);

  void xaxissetup(){
  
  xaxis.setMaxSpeed(5000.0); // Set the maximum speed of the stepper motor
  millis(); // Not sure why this line is here, it doesn't have any effect
  pinMode(right_limit_pin, INPUT_PULLUP);
  pinMode(left_limit_pin, INPUT_PULLUP);
    }
  void xaxisloop ()
  {
     
    float xValue = analogRead(joyXPin); // Read the X-axis value from the joystick
   int  rightstate = digitalRead(right_limit_pin);
   int   leftstate = digitalRead(left_limit_pin);
    if ((xValue > 1000)&&(rightstate==1)) { // If the X-axis value is greater than 800
      xaxis.setSpeed(x); // Set the speed of the stepper motor to a positive value
      xaxis.runSpeed(); // Run the stepper motor at the set speed
    } 
else 
    if ((xValue < 100)&&(leftstate==1)) { // If the X-axis value is less than 200
      xaxis.setSpeed(-(x)); // Set the speed of the stepper motor to a negative value
      xaxis.runSpeed(); // Run the stepper motor at the set speed
    } 
else {
      xaxis.stop(); // Stop the stepper motor if the X-axis value is between 200 and 800
      
    }
  }
