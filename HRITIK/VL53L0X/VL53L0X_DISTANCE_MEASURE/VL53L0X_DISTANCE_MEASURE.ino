#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(1000);
  sensor.startContinuous(10);
}

void loop()
{
  double distance =sensor.readRangeContinuousMillimeters();
  String output = "Distance: " + String((distance/10)-3.20) + " cm";
  Serial.println(output);
  
  delay(1000);
} 