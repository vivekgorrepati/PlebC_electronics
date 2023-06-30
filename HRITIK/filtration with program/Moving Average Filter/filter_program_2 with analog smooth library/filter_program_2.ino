#include <AnalogSmooth.h>
 
int analogPin = 1;
 
// Defaults to window size 10
AnalogSmooth as = AnalogSmooth();
 
// Window size can range from 1 - 100
AnalogSmooth as100 = AnalogSmooth(50);
 
void setup() {
  Serial.begin(9600);
}
 
void loop() {
  // Regular reading
  float analog = analogRead(analogPin);
  Serial.print("Non-Smooth: ");
  Serial.println(analog);
   
  // Smoothing with window size 10
  float analogSmooth = as.smooth(analog);
  Serial.print("Smooth (10): "+);  
  Serial.println(analogSmooth);
 
  // Smoothing with window size 100
  float analogSmooth100 = as100.analogReadSmooth(analogPin);
  Serial.print("Smooth (100): ");  
  Serial.println(analogSmooth100);
 
  Serial.println("");
  delay(1000);
}