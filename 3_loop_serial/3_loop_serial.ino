int driverPUL1 = 11;    // PUL- pin
int driverPUL2 = 10;    // PUL- pin
int driverPUL3 = 9;    // PUL- pin
int driverDIR1 = 8;    // DIR- pin
int driverDIR2 = 7;    // DIR- pin
int driverDIR3 = 6;    // DIR- pin

void setup() {
  // put your setup code here, to run once:
  pinMode (driverPUL1, OUTPUT);
  pinMode (driverDIR1, OUTPUT);
  pinMode (driverPUL2, OUTPUT);
  pinMode (driverDIR2, OUTPUT);
  pinMode (driverPUL3, OUTPUT);
  pinMode (driverDIR3, OUTPUT);
  Serial.begin(9600);
  
}

void loop ()
{
  loop1();
  loop2();
  loop3();
  }
void loop1() {
  // put your main code here, to run repeatedly:
 int delays1 = 300;
    digitalWrite(driverPUL1,LOW);
    Serial.print("delays1 low = ");
    Serial.println(delays1);
    delayMicroseconds(delays1);
    digitalWrite(driverPUL1,HIGH);
    Serial.print("delays1 high = ");
    Serial.println(delays1);
    delayMicroseconds(delays1);
  
}

void loop2() {
  // put your main code here, to run repeatedly:
 int delays2 = 400;
    digitalWrite(driverPUL2,LOW);
    Serial.print("delays2 low = ");
    Serial.println(delays2);
    delayMicroseconds(delays2);
    digitalWrite(driverPUL2,HIGH);
    Serial.print("delays2 high = ");
    Serial.println(delays2);
    delayMicroseconds(delays2);
}

void loop3() {
  // put your main code here, to run repeatedly:
 int delays3 = 500;
    digitalWrite(driverPUL3,LOW);
    Serial.print("delays3 low = ");
    Serial.println(delays3);
    delayMicroseconds(delays3);
    digitalWrite(driverPUL3,HIGH);
    Serial.print("delays3 high = ");
    Serial.println(delays3);
    delayMicroseconds(delays3);
}
