// Get the current time
#include <AccelStepper.h>
 int upState = 0;
int dnState = 0;

//int upperstate = 1;
//int lowerstate = 1;

const int joyYPin = A1; // Analog input pin for joystick X-axis

float y = 1000; // Acceleration value

#define ySTEP_PIN 10
#define yDIR_PIN 9
#define upper_limit_pin 5
#define lower_limit_pin 6

AccelStepper yaxis(AccelStepper::DRIVER, ySTEP_PIN, yDIR_PIN);
  void yaxissetup(){
  
  yaxis.setMaxSpeed(5000.0); // Set the maximum speed of the stepper motor
  millis(); // Not sure why this line is here, it doesn't have any effect
  pinMode(upper_limit_pin, INPUT_PULLUP);
  pinMode(lower_limit_pin, INPUT_PULLUP);
    }
  void yaxisloop ()
  {
     
    float yValue = analogRead(joyYPin); // Read the X-axis value from the joystick
   int   upperstate = digitalRead(upper_limit_pin);
   int   lowerstate = digitalRead(lower_limit_pin);
    if ((yValue > 1000)&&(upperstate==1)) { // If the X-axis value is greater than 800
      yaxis.setSpeed(y); // Set the speed of the stepper motor to a positive value
      yaxis.runSpeed(); // Run the stepper motor at the set speed
    } 
else 
    if ((yValue < 100)&&(lowerstate==1)) { // If the X-axis value is less than 200
      yaxis.setSpeed(-(y)); // Set the speed of the stepper motor to a negative value
      yaxis.runSpeed(); // Run the stepper motor at the set speed
    } 
else {
      yaxis.stop(); // Stop the stepper motor if the X-axis value is between 200 and 800
      
    }
  }
