#include "HX711.h"

HX711 scale(6, 5); //HX711 scale(6, 5);

float calibration_factor = -259;
float units;
float ounces;
float gram_force;
float newton;

void setup()
{
  Serial.begin(9600);
  Serial.println("HX711 weighing");
  scale.set_scale(calibration_factor);
  scale.tare();
  Serial.println("Readings:");
}

void loop()
{
  Serial.print("Reading:");
  units = scale.get_units(),10;
  if (units < 0)
  {
    units = 0.00;
  }
  ounces = units * 0.035274;

  newton = (units/1000)*9.81;
  String output_weight_force = "Weight\t" + String(units) + " grams" + "\t" + String(newton) + " N" ;
  Serial.println(output_weight_force);


  delay(1000);
}
