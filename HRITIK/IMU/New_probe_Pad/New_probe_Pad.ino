// https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/#:~:text=To%20use%20the%20two%20I2C%20bus%20interfaces%20of%20the%20ESP32,to%20create%20two%20TwoWire%20instances.&text=Then%2C%20initialize%20I2C%20communication%20on,pins%20with%20a%20defined%20frequency.&text=Then%2C%20you%20can%20use%20the,with%20the%20I2C%20bus%20interfaces.
// https://quadmeup.com/esp32-and-multiple-i2c-buses/
// https://github.com/stm32duino/Arduino_Core_STM32/wiki/API

/*SPARKFUN HX711
  
   Connections
   =============
   connect digital pin PB5 to HX711 CLK
   connect digital pin PB6 to HX711 DOUT
   Connect VDD to 5V 
   Connect GROUND to common ground
*/

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog B10
   Connect SDA to analog B3
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   03-08-2023   - 60 SPS(samples per second) of BNO055
   04-01-2024 - Sparkfun Hx711 and BNO055, app.175 SPS, 85+85 SPS each
*/
#include "HX711.h"  //use sparkfun library 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Set pins for HX711
#define DOUT PB6 
#define CLK  PB5

// Arduino Nano's I2C address
#define NANO_ADDRESS 0x42 

//Set pins for I2C2
#define I2C2_SDA_PIN PB4
#define I2C2_SCL_PIN PA8

#define calibration_factor -125000 //This value is obtained using the SparkFun_HX711_Calibration sketch
HX711 scale;

#define I2C_Address 0x42
#define MAX_REMOTE_DATA_LENGTH 50 // Maximum length of the string
char remoteData[MAX_REMOTE_DATA_LENGTH]; // Buffer to store the received string


TwoWire Wire2(PB3, PB10);

float lastw = 0,lastx =0,lasty =0,lastz = 0;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
                                  
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
unsigned long prevTimeQuat = millis();
unsigned long prevTimePressure = millis();
unsigned long prevTimeRemote = millis();

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


void setup(void) {
      Serial.begin(115200); 

      Wire2.begin(); // Initialize I2C2 with PB7 (SDA) and PB6 (SCL)

      scale.begin(DOUT, CLK);
      scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
      scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0

      initialimu();
      millis();
      remote();

  }

void loop(void) {
  
   if (millis() - prevTimeQuat  >8) // Check if enough time has passed to read quaternion data
   {
       quaternion();   // Read quaternion data from the BNO055 sensor
       prevTimeQuat = millis(); // Store the current time for the next comparison
   }
  
  if (millis() - prevTimePressure  >10) // Check if enough time has passed to read pressure (weight) data
   {
       pressure(); // Read pressure (weight) data from the load cell
       prevTimePressure = millis(); // Store the current time for the next comparison
   }
   if (millis() - prevTimeRemote  >10) // Check if enough time has passed to read pressure (weight) data
   {
       remote(); // Read pressure (weight) data from the load cell
       prevTimeRemote = millis(); // Store the current time for the next comparison
   }

  }


void initialimu(){
   pinMode(LED_PIN, OUTPUT);

  while (!Serial) 
  delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); 


  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // delay(500);

  }


void quaternion(){
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();

  if (lastw == quat.w()&&lastx==quat.x()&&lasty==quat.y()&&lastz==quat.z())
  {}
  else{

    String qString = "quat value\t$" + String(quat.w(), 4) + "$" + String(quat.x(), 4) + "$" + String(quat.y(), 4) + "$" + String(quat.z(), 4);
    Serial.println(qString);
    // delay(1);

    }

    // Quaternion data
  lastw = quat.w();
  lastx = quat.x();
  lasty = quat.y();
  lastz = quat.z();
  }


void pressure(){  
  float weight_lbs = scale.get_units();
  int weight_grams = weight_lbs * 453.592; // 1 lbs = 0.453592 kg
  // Serial.print("Reading: ");
  Serial.println(-weight_grams); // Display weight in grams with 1 decimal place
  }

void remote(){
    int i = 0;
  
  // Request the entire string from the I2C device
  Wire2.requestFrom(I2C_Address, MAX_REMOTE_DATA_LENGTH);

  // Read characters until buffer is full or end of transmission is reached
  while (Wire2.available() > 0 && i < MAX_REMOTE_DATA_LENGTH - 1) {
    char c = Wire2.read(); // Read one byte
    remoteData[i] = c; // Store the byte in the buffer
    i++;
  }

  remoteData[i] = '\0'; // Null-terminate the string

  // Print the received string
  Serial.println(remoteData);
    
   }

  

