/*HX711
  
   Connections
   =============
   connect digital pin 5 to HX711 CLK
   connect digital pin 6 to HX711 DOUT
   Connect VDD to 5V 
   Connect GROUND to common ground
*/

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   03-08-2023   - 60 SPS(samples per second) of BNO055
*/
#include "HX711.h" 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
 
HX711 scale(PB6, PB5);     //(HX711 DOUT, HX711 CLK)

float calibration_factor = -259;  //Change calibration factor here
float units;
float ounces;
float gram_force;
float newton;
float lastw = 0,lastx =0,lasty =0,lastz = 0;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
                                  
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
unsigned long prevTimeQuat = millis();
unsigned long prevTimePressure = millis();

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup(void) {
      Serial.begin(115200); 
      initialloadcell();
      initialimu();
      millis();

}

void loop(void) {
  
   if (millis() - prevTimeQuat  >0) // Check if enough time has passed to read quaternion data
   {
       quaternion();   // Read quaternion data from the BNO055 sensor
       prevTimeQuat = millis(); // Store the current time for the next comparison
   }
  
  if (millis() - prevTimePressure  >100) // Check if enough time has passed to read pressure (weight) data
   {
       pressure(); // Read pressure (weight) data from the load cell
       prevTimePressure = millis(); // Store the current time for the next comparison
   }

}

void initialimu()
{
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

void initialloadcell()
  {
    scale.set_scale(calibration_factor);
    scale.tare();
  }

void pressure(){
    
  units = scale.get_units(),10;
  if (units < 0)
  {
    units = 0.00;
  }
  newton = (units/1000)*9.81;
  // String output_weight_force = "Weight\t" + String(units) + " grams" + "\t" + String(newton) + " N" ;
  String output_weight_force = String(newton) + " N" ;
  Serial.println(output_weight_force);

     }