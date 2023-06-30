int driverPUL = 9;    // PUL- pin
int driverDIR1 = 6;    // DIR- pin

long int pulseTime, delays,k;

 long int rpm = 94;
 long int ppr = 10 + 1600;
 

 boolean setdir1 = HIGH; // Set Direction

 void setup() {

  pinMode (driverPUL, OUTPUT);
  pinMode (driverDIR1, OUTPUT);

// Serial.begin(9600);
}

void loop() {
   
    pulseTime=60000000/(ppr*rpm);
    delays=pulseTime/2;
    // Serial.println(delays);
    
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(delays);
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(delays);
}