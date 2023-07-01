#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

int counter=0;
int count=10;
double sum=0;
double av;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  sensor.setTimeout(1000);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous(10);
}

void loop()
{
  double distance =sensor.readRangeContinuousMillimeters();
// distance = distance -45;// -55 is to compensate for error. Change or set it to zero to make it work for your sensor
  Serial.print("Distance: ");
 Serial.print(distance);
//  Serial.print("mm");
  counter++;
  if (counter==10) 
  {
for(int i=0; i<count; i++)
{
  sum = 0;
  sum=sum+distance;
  counter=0;
}
av = sum / count;
//Serial.print("AVERAGE");
Serial.print("AVERAGE Distance: ");
Serial.println(av);
//Serial.print("mm");

  }
  Serial.println();
  delay(1000);
} 
