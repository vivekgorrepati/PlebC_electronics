unsigned long prevTime = millis(); // Variable to store the previous time


#include "xaxis.h"
#include "yaxis.h"
#include "zaxis.h"



void setup() {
   Serial.begin(9600);
  xaxissetup();
  yaxissetup();
  zaxissetup();
}

void loop() {
 unsigned long currentTime = millis(); 
     if (currentTime - prevTime > 0) {
   xaxisloop();
   yaxisloop();
   zaxisloop();
  currentTime = prevTime;
}}
