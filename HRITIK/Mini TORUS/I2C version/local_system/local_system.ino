#include <AccelStepper.h>
#include <Wire.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution = 2000;

#define I2C_Address 0x14
#define MAX_BEST_ANGLES_LENGTH 32 // Maximum length of the string
char receivedMessage[MAX_BEST_ANGLES_LENGTH]; // Buffer to store the received message
#define TIMEOUT_MS 1000           // Timeout in milliseconds

// Gear ratio for each motor
const float gearRatio_1 = 1.0; // Gear ratio for motor 1
const float gearRatio_2 = 1.0; // Gear ratio for motor 2
const float gearRatio_3 = 1.0; // Gear ratio for motor 3

// Create instances of the AccelStepper class for each motor
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
AccelStepper stepper1(AccelStepper::FULL4WIRE, 6, 8, 7, 9); // Motor 1 - Wire connections
AccelStepper stepper2(AccelStepper::FULL4WIRE, 2, 4, 3, 5); // Motor 2 - Wire connections
AccelStepper stepper3(AccelStepper::FULL4WIRE, 10, 12, 11, 13); // Motor 3 - Wire connections

// Variables to store the angles for each motor
float angle_1 = 0;
float angle_2 = 0;
float angle_3 = 0;

// unsigned long prevTimeMainLoop = millis();
// unsigned long prevTimeBestAngles = millis();

void setup()
{
    Serial.begin(9600);

    // Set up each stepper motor
    stepper1.setMaxSpeed(6000); // Set maximum speed in steps per second
    stepper1.setAcceleration(600); // Set acceleration in steps per second per second

    stepper2.setMaxSpeed(6000);
    stepper2.setAcceleration(600);

    stepper3.setMaxSpeed(6000);
    stepper3.setAcceleration(600);

    Wire.begin(); // Initialize I2C2 with PB7 (SDA) and PB6 (SCL)
}

// void loop(void){
//   // if (millis() - prevTimeMainLoop  >4) // Check if enough time has passed to read quaternion data
//   //  {
//   //      mainLoop();   // Read quaternion data from the BNO055 sensor
//   //      prevTimeMainLoop = millis(); // Store the current time for the next comparison
//   //  }
  
//   if (millis() - prevTimeBestAngles  >10) // Check if enough time has passed to read pressure (weight) data
//    {
//        bestAngles(); // Read pressure (weight) data from the load cell
//        prevTimeBestAngles = millis(); // Store the current time for the next comparison
//    }
  
// }

void loop()
{
    bestAngles();
    
    // String receivedString = Serial.readStringUntil('\n');
  
    Serial.println(receivedMessage);
    String receivedString = receivedMessage;
    
    // Split the received string into angle values
    char *token;
    char *angleStr;
    char receivedChars[receivedString.length() + 1];
    strcpy(receivedChars, receivedString.c_str());

    // Process the first angle separately
    token = strtok(receivedChars, "$");
    angleStr = token;
    angle_1 = atof(angleStr);

    // Process the remaining angles
    token = strtok(NULL, "$");
    int angleIndex = 2;
    while (token != NULL)
    {
        angleStr = token;
        float angle = atof(angleStr);

        // Assign angles to the corresponding motors
        switch (angleIndex)
        {
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

    // Update all motors
    stepper1.run();
    stepper2.run();
    stepper3.run();

    // Check if each motor has reached its target position
    if (!stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning())
    {
        // All motors have reached their target positions
        // Do something if needed after all motors have stopped moving
    }
}

void bestAngles()
{
    
    int messageLength = 0;                         // Length of the received message
    bool withinMessage = false;                    // Flag to track whether we are within the message

    // Request the entire string from the I2C device
    Wire.requestFrom(I2C_Address, MAX_BEST_ANGLES_LENGTH, true); // Request with stop signal

    unsigned long startMillis = millis(); // Start time for timeout

    // Read characters until buffer is full, end of transmission is reached, or '*' is encountered
    while (Wire.available() && messageLength < MAX_BEST_ANGLES_LENGTH - 1)
    {
        char c = Wire.read(); // Read one byte
        if (c == '#')
        {
            withinMessage = true; // Start of message
            continue;             // Skip '#' character
        }
        else if (c == '*')
        {
            break; // End of message reached
        }
        if (withinMessage)
        {
            receivedMessage[messageLength++] = c; // Store the byte in the buffer
        }
        // Check for timeout
        if (millis() - startMillis > TIMEOUT_MS)
        {
            Serial.println("Timeout occurred while waiting for data.");
            break; // Break if timeout occurs
        }
    }

    // Trim trailing whitespace characters
    while (messageLength > 0 && isspace(receivedMessage[messageLength - 1]))
    {
        messageLength--;
    }

    receivedMessage[messageLength] = '\0'; // Null-terminate the string

    //Print the received message if it's not empty
    if (messageLength > 0)
    {
        String LocalSystemString = "Angles : " + String(receivedMessage);
        Serial.println(LocalSystemString);
    }
  
}
