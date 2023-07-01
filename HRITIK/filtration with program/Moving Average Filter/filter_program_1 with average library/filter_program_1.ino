#include <averager.h>
float alpha = 0.05;
averager<float> average;
void setup() {
  Serial.begin(9600);
  // Default is 0.1, so must be changed if we needs a different alpha
  average.setExponentialMovingAverageAlpha(alpha);
}
void loop() {
  uint16_t new_value = analogRead(A0);
  average.updateExponentialMovingAverage(new_value);
  Serial.println(average.getExponentialMovingAverage());
  delay(100);
}