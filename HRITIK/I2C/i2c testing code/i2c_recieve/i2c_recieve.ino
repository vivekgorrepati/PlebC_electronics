#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Trigger Switch
#define triggerSwitch PB9

#define I2C_Address 0x08
#define MAX_REMOTE_DATA_LENGTH 25 // Maximum length of the string
//char remoteData[MAX_REMOTE_DATA_LENGTH]; // Buffer to store the received string
#define TIMEOUT_MS 1000 // Timeout in milliseconds
//#define END_OF_STRING_CHAR '\0' // Special character indicating end of string

TwoWire Wire2(PB4, PA8); //(sda, scl)

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);


unsigned long prevTimeQuat = millis();
unsigned long prevTimeRemote = millis();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Wire2.begin(); // Initialize I2C2 with PB7 (SDA) and PB6 (SCL)
  pinMode(triggerSwitch, INPUT_PULLUP);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(500);

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

      valueimu();
      millis();
      remote();
  }



void loop(void)
{
 if (millis() - prevTimeQuat  >8) // Check if enough time has passed to read quaternion data
   {
       valueimu();   // Read quaternion data from the BNO055 sensor
       prevTimeQuat = millis(); // Store the current time for the next comparison
   }

   if (millis() - prevTimeRemote  >10) // Check if enough time has passed to read pressure (weight) data
   {
       remote(); // Read pressure (weight) data from the load cell
       prevTimeRemote = millis(); // Store the current time for the next comparison
   }
 // delay(1000);
  
}

void valueimu(){

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    
      // Quaternion data
      imu::Quaternion quat = bno.getQuat();
      if(digitalRead(triggerSwitch) == 0){
      Serial.print("qW: ");
      Serial.print(quat.w(), 4);
      Serial.print(" qX: ");
      Serial.print(quat.x(), 4);
      Serial.print(" qY: ");
      Serial.print(quat.y(), 4);
      Serial.print(" qZ: ");
      Serial.print(quat.z(), 4);
      Serial.println("\t\t");
   

      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
    
    
  }

void remote(){
  char receivedMessage[MAX_REMOTE_DATA_LENGTH]; // Buffer to store the received message
  int messageLength = 0; // Length of the received message
  bool withinMessage = false; // Flag to track whether we are within the message

  // Request the entire string from the I2C device
  Wire2.requestFrom(I2C_Address, MAX_REMOTE_DATA_LENGTH, true); // Request with stop signal

  unsigned long startMillis = millis(); // Start time for timeout

  // Read characters until buffer is full, end of transmission is reached, or '*' is encountered
  while (Wire2.available() && messageLength < MAX_REMOTE_DATA_LENGTH - 1) {
    char c = Wire2.read(); // Read one byte
    if (c == '#') {
      withinMessage = true; // Start of message
      continue; // Skip '#' character
    } else if (c == '*') {
      break; // End of message reached
    }
    if (withinMessage) {
      receivedMessage[messageLength++] = c; // Store the byte in the buffer
    }
    // Check for timeout
    if (millis() - startMillis > TIMEOUT_MS) {
      Serial.println("Timeout occurred while waiting for data.");
      break; // Break if timeout occurs
    }
  }

  // Trim trailing whitespace characters
  while (messageLength > 0 && isspace(receivedMessage[messageLength - 1])) {
    messageLength--;
  }

  receivedMessage[messageLength] = '\0'; // Null-terminate the string

  // Print the received message if it's not empty
  if (messageLength > 0) {
    String gantryString = "Gantry_values : " + String(receivedMessage) ;
    Serial.println(gantryString);
  }

}
