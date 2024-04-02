//for loop 2

#include <AccelStepper.h>
#include <Wire.h>

void requestEncoderValue();

const int slaveAddress = 0x11; // Address of the slave device
const int MAX_REMOTE_DATA_LENGTH = 15; // Maximum expected length of the string from the slave

// Define the stepper motor connections
#define stepPin 9
#define dirPin 8
#define enablePin 10

float motorSteps =  10000;
float encoder_PPR_Values = 2400;

long encoderCurrentValue = 0;

float oneStep = encoder_PPR_Values/motorSteps;
float oneValue = motorSteps/encoder_PPR_Values;
float oneDegree = encoder_PPR_Values/360 ;

float currentSteps = 0;
float angleToValue;
float targetSteps;
int stepsToMove = 0;
float angle = 0;


// Initialize the AccelStepper library
// For a driver, the interface type is 1 when using a driver.
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);
int intAngleRecieved;
void setup() {
  Serial.begin(115200);

  Wire.begin(); // Start I2C as master

  pinMode(enablePin, OUTPUT); // Set the enable pin as output
  digitalWrite(enablePin, LOW); // Enable the motor

  stepper.setMaxSpeed(50000); // Set max speed in steps per second, adjust as needed
  stepper.setAcceleration(5000); // Set acceleration in steps per second^2, adjust as needed
}

void loop(){

  for (int i = 0; i < 100; i++) {
    intAngleRecieved = random(-360, 361); // Generate a random angle between -360 and 360
    Serial.print(i);
    Serial.print(" - Generated Angle: ");
    Serial.println(intAngleRecieved);

    process(intAngleRecieved);
    
  }
  // Return to 0 degrees after processing 100 random angles
  Serial.println("Returning to 0 degrees...");
  process(0);
  Serial.println("Returned to 0 degrees.");

  // Halt further execution
  while (true) {
    delay(20); // Effectively pauses the program with a long delay
  }

}

void process(int intAngleRecieved) 
  {
    
      if(intAngleRecieved <= 360)
      {
        // Serial.print("Angle Recieved : ");
        // Serial.println(intAngleRecieved);

        requestEncoderValue();
        // Serial.print("encoder current Value : ");
        // Serial.println(encoderCurrentValue);

        
        int motorCurrrentAngle = encoderCurrentValue * 0.15 ;
        // Serial.print("motor current angle : ");
        // Serial.println(motorCurrrentAngle);

        currentSteps = encoderCurrentValue * oneValue ;

        // Serial.print("current Steps : ");
        // Serial.println(currentSteps);


        angleToValue = intAngleRecieved * oneDegree ;
        // Serial.print("angle to value : ");
        // Serial.println(angleToValue);

        targetSteps = angleToValue * oneValue ;
        // Serial.print("target steps : ");
        // Serial.println(targetSteps);

        stepsToMove = targetSteps - currentSteps ;  
        // Serial.print("steps to move : ");
        // Serial.println(stepsToMove);

        long targetEncoderValue = intAngleRecieved * 6.6666666666666666666 ;
        // Serial.print("target Encoder Value : ");
        // Serial.println(targetEncoderValue);

        stepper.moveTo(stepper.currentPosition() + stepsToMove);
        while (stepper.distanceToGo() != 0) 
        {
          stepper.run(); // Continuously run the stepper motor to the target position
        }

        // After reaching the initial target, adjust based on actual position
        bool adjustmentNeeded = true;
        while (adjustmentNeeded) //adjustment while loop start
          {
              requestEncoderValue();
      
              // Calculate difference between target and current positions
              long positionDifference = targetEncoderValue - encoderCurrentValue;
        
              // If difference is non-zero, adjust position
              if (positionDifference != 0) 
                {
                    stepper.move(positionDifference * oneValue); // Adjust by moving the difference
                    while (stepper.distanceToGo() != 0) 
                      {
                        stepper.run();
                      }
                } 
                else 
                  {
                    adjustmentNeeded = false; // Exit loop if no adjustment needed
                  }

                    // Add a short delay for stability, if necessary
                    delay(20);
          }// adjustment while loop close     


      }
  }
  
  
  
void requestEncoderValue()
 {
  Wire.requestFrom(slaveAddress, sizeof(long)); // Request sizeof(long) bytes from the slave
  while (Wire.available() < sizeof(long)) {
    // Wait for the data to be available
  } 
  
  Wire.readBytes((char*)&encoderCurrentValue, sizeof(encoderCurrentValue)); // Read the incoming bytes into encoderCurrentValue

  // Serial.print("Encoder Value: ");
  // Serial.println(encoderCurrentValue);
  }


