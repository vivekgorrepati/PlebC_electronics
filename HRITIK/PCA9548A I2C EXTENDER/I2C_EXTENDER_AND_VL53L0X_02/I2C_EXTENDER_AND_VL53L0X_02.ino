#include <Wire.h>
#include <VL53L0X.h>

#define PCAADDR 0x70

VL53L0X sensor;

void pcaselect(uint8_t bus) {
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void setup() {
 
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Ready");

  sensor.init();
  sensor.setTimeout(1000);
  sensor.startContinuous();

  pcaselect(0);
}

void loop() {
  double distance = sensor.readRangeContinuousMillimeters();
  
  String output = "Distance: " + String((distance/10)-3.20) + " cm";
  Serial.println(output);
  
  delay(100);
}
