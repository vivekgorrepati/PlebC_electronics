unsigned long prevTime = millis();
int ena = 5;
int driverPUL = 11;    // PUL- pin
int driverDIR = 8;    // DIR- pin
int upperLimit = 2;
int lowerLimit = 3;
int delay1(0);
boolean motorDirection = true;

void setup() {

  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR, OUTPUT);
  
  pinMode (upperLimit, INPUT_PULLUP);
  pinMode (lowerLimit, INPUT_PULLUP);

//Serial.begin(9600);
millis();
}

void loop() 
   { 
    unsigned long currentTime = millis();
   if (currentTime - prevTime  >0)
   {
    boolean upperLimitState = digitalRead(upperLimit);
    boolean lowerLimitState = digitalRead(lowerLimit);

Serial.print("motorDirection = ");
Serial.println(motorDirection);

Serial.print("upperLimitState = ");
Serial.println(upperLimitState);
Serial.print("lowerLimitState = ");
Serial.println(lowerLimitState);
//clockwise();
//delay(10);

    if (!upperLimitState && motorDirection) 
        {
          motorDirection = false; 
          Serial.print(" in if motodir = ");
          Serial.println( motorDirection);
          stopMotor();
          delay(delay1); 
          clockwise(); // Change direction to reverse
        }
    else if (!lowerLimitState && !motorDirection) 
        {
          motorDirection = true; 
          Serial.print(" in else if motordir = ");
          Serial.println( motorDirection);
          stopMotor();  
          delay(delay1); 
          anticlockwise();// Change direction to forward
        }
    else 
        {
          Serial.print(" in else  motordir = ");
          Serial.println( motorDirection);
        stopMotor();
        

    if(motorDirection)
    {
      Serial.println( " turning anti clockwise");
      anticlockwise();
      delay (delay1);
      pulse();
    }
    else
    {
      Serial.println( " turning clockwise");
      clockwise();
      delay (delay1);
      pulse();
     }
    }}}

  void clockwise() 
    { 
      digitalWrite(driverDIR, HIGH);
      pulse();
    }
  void anticlockwise() 
    { 
      digitalWrite(driverDIR,LOW);
      pulse();
    }
  void stopMotor() 
    {
      digitalWrite(ena, HIGH);
      digitalWrite(driverPUL, LOW);
    }
  void pulse() 
    {
      int delays = 180;
      digitalWrite(driverPUL,LOW);
      delayMicroseconds(delays);
      digitalWrite(driverPUL,HIGH);
      delayMicroseconds(delays);
    }
    
