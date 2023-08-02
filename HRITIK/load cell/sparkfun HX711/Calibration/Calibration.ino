#include "HX711.h"

#define DOUT 3
#define CLK 2

HX711 scale;

float calibration_factor = -190000; // -7050 worked for my 440lb max scale setup

void setup() {
  Serial.begin(9600);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare(); // Reset the scale to 0

  long zero_factor = scale.read_average(); // Get a baseline reading
  Serial.print("Zero factor: "); // This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
}

void loop() {
  scale.set_scale(calibration_factor); // Adjust to this calibration factor

  Serial.print("Reading: ");
  float weight_lbs = scale.get_units();
  // Serial.print(weight_lbs, 1); // Display weight in pounds with 1 decimal place
  // Serial.print(" lbs");

  // Convert lbs to grams
  int weight_grams = weight_lbs * 453.592;

  //Serial.print(" (");
  Serial.print(weight_grams); // Display weight in grams with 1 decimal place
  Serial.print(" grams");

  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();

  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a')
      calibration_factor += 10;
    else if (temp == '-' || temp == 'z')
      calibration_factor -= 10;
  }
  delay(300);
}
