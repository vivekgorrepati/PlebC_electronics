int encoderPin1 = 2;
int encoderPin2 = 3;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;
float encoderValueDegrees = 0;
float distance = 0;

int lastMSB = 0;
int lastLSB = 0;

void setup() {
  Serial.begin (9600);

  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

}

void loop(){ 
  // Read the encoder value in degrees
  encoderValueDegrees = encoderValue * 0.15;  //0.075; // Using the conversion factor = 180/2400= 0.075, Adjusted for a 1:2 ratio (360/2 = 180 degrees per count)
  distance = encoderValue * 0.0395833333333333;// Read the encoder value in mm

  Serial.print("Value : ");
  Serial.println(encoderValue);
  Serial.print("Degree : ");
  Serial.println(encoderValueDegrees);
  Serial.print("Distance : ");
  Serial.print(distance + 60.22);
  Serial.println(" mm");
  delay(200); //just here to slow down the output, and show it will work  even during a delay
}


void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
  
}
