// defines pins numbers
const int STEP_PIN = 3; 
const int DIR_PIN = 4; 
const int enpin = 5;
int angle = 0;
int RotationRequired;

int y;
int steps;

void setup() {
  // Sets the two pins as Outputs
  pinMode(STEP_PIN,OUTPUT); 
  Serial.begin(9600);
  pinMode(DIR_PIN,OUTPUT);
  Serial.println("Type the angle of rotation:");
}
void loop() {
  /*digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  
  if(Serial.available()>0)
  {Serial.println("Hello");
  digitalWrite(dirPin,HIGH);
  angle = Serial.readString().toInt();
  Serial.println(angle);
  steps = angle/1.8;
  Serial.println(steps);
  for(int x = 0; x <steps; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
  delay(1000);
  }// One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  int y = Serial.readString().toInt()/1.8;
  for(int x = 0; x < y; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(500);
  }
  delay(1000);*/


  if (Serial.available()>0)
{
  y = Serial.parseFloat();
  Serial.println(y);

  if(y<0)
  {
    digitalWrite(DIR_PIN, LOW);
  }

  
  if(y>0)
  {
    digitalWrite(DIR_PIN, HIGH);
  }
  
  
  /*Serial.println("One Rotation =");
  Serial.print(oneRotation);*/
  RotationRequired = abs(y)/1.8;  Serial.print("RotationRequired =");
  Serial.println(RotationRequired);
  
  //digitalWrite(EN_PIN, LOW);
   
  
  steps = RotationRequired;
  Serial.print("steps=");
  Serial.println(steps);
  
  for(int i=1;i <=steps ; i++)
  {Serial.println("Entered LOOP");
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(STEP_PIN, LOW);
  
  }
  //digitalWrite(EN_PIN, HIGH);


}
}
