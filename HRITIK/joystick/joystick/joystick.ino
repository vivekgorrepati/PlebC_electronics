#define VRX_PIN  A0 // Arduino pin connected to VRX pin
#define VRY_PIN  A1 // Arduino pin connected to VRY pin

int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis

void setup() {
  Serial.begin(9600) ;
}

void loop() {
  // read analog X and Y analog values
  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);

  //print data to Serial Monitor on Arduino IDE
  // Serial.print("x = ");
  // Serial.print(xValue);
  // Serial.print(", y = ");
  // Serial.println(yValue);
  // delay(200);

  //Code for joystick normal position
  if(xValue == 497 && yValue == 498)
  {
    Serial.println("Normal Position");
  }

  //Code for X-axis
  if(xValue < 495 && yValue == 498)
  {
    Serial.println("Moving Left");
  
  }

  if(xValue > 500 && yValue == 498)
  {
    Serial.println("Moving Right");
  
  }

  if(yValue >520 && xValue == 497)
  {
    Serial.println("Moving Downward");
    
  }

  // Code for y-axis
  if(yValue < 495 && xValue ==497)
  {
    Serial.println("Moving Upward");
    
  }
  //Code for z-axis
  if(xValue < 490 && yValue < 490)
  {
    Serial.println("Moving Upward on z-axis");
  }

  if(xValue >510 && yValue > 510)
  {
    Serial.println("Moving Upward on z-axis");
  }

  delay(300);

}
