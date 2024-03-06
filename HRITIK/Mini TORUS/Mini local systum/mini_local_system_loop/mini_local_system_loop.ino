#include <AccelStepper.h>

// Define the number of steps per revolution for your stepper motor
const int stepsPerRevolution = 2000;

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

// List of angle values
String strings_to_send[] = {
                    "20$40$20",
                    "30$60$30",
                    "40$80$40",                    
                    "50$100$50",
                    "60$120$60",
                    "70$140$70",
                    "80$160$80",
                    "90$180$90",
                    "100$200$100",
                    "110$220$110",
                    "120$240$120",                    
                    "130$260$130",
                    "140$280$140",
                    "150$300$150",
                    "160$320$160",
                    "170$340$170",
                    "180$360$180",
                    "170$340$170",
                    "160$320$160",                    
                    "150$300$150",
                    "140$280$140",
                    "130$260$130",
                    "120$240$120",
                    "110$220$110",
                    "90$180$90",
                    "80$160$80",
                    "70$140$70",                    
                    "60$120$60",
                    "50$100$50",
                    "40$80$40",
                    "30$60$30"
                    "20$40$20",
                    "10$20$10",
                    "0$0$0"
};

// Index for accessing strings_to_send
int listIndex = 0;

// Variables to manage timing
unsigned long previousMillis = 0;
const long interval = 2000; // 2 seconds

// State variable for managing state machine
enum State {
    IDLE,
    MOVE_MOTORS
};

State currentState = IDLE;

void setup() {
    Serial.begin(9600);

    // Set up each stepper motor
    stepper1.setMaxSpeed(6000);    // Set maximum speed in steps per second
    stepper1.setAcceleration(600); // Set acceleration in steps per second per second

    stepper2.setMaxSpeed(6000);
    stepper2.setAcceleration(600);

    stepper3.setMaxSpeed(6000);
    stepper3.setAcceleration(600);
}

void loop() {
    unsigned long currentMillis = millis();

    switch (currentState) {
        case IDLE:
            // Check if it's time to move to the next set of angles
            if (currentMillis - previousMillis >= interval) {
                currentState = MOVE_MOTORS;
            }
            break;

        case MOVE_MOTORS:
            // Move motors to the next set of angles
            processNextValue();
            // Update previousMillis for next interval
            previousMillis = currentMillis;
            // Move back to IDLE state
            currentState = IDLE;
            break;
    }

    // Update all motors
    stepper1.run();
    stepper2.run();
    stepper3.run();

    // Check if each motor has reached its target position
    if (!stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning()) {
        // All motors have reached their target positions
        // Do something if needed after all motors have stopped moving
    }

     // Reset listIndex if it reaches the end of the list
       if (listIndex >= sizeof(strings_to_send) / sizeof(strings_to_send[0])) {
        listIndex = 0;
    }
}

// Function to read the next value from the list and move the motors accordingly
void processNextValue() {
    if (listIndex < sizeof(strings_to_send) / sizeof(strings_to_send[0])) {
        String receivedString = strings_to_send[listIndex++];

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
        while (token != NULL) {
            angleStr = token;
            float angle = atof(angleStr);

            // Assign angles to the corresponding motors
            switch (angleIndex) {
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

    }
}
