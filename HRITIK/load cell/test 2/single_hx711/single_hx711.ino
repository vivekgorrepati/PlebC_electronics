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

  //gram_force = (units/1000)*9.80665; //formula of Gram-force (gf) = Grams (g) × Acceleration due to gravity (9.80665 m/s²) 
  //newton = gram_force*0.009807; //formula of newtons = gram-force × 0.009807
  newton = (units/1000)*9.81;

  Serial.print(units);
  Serial.print(" grams"); 
  Serial.print("\t");
  Serial.print(newton);
  Serial.println(" N");

  delay(1000);
}
