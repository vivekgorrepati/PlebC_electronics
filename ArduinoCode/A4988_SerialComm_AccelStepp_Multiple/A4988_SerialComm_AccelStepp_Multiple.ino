


#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>



// Include the AccelStepper Library
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
boolean Execute = false;
boolean HomeFlag = false;

unsigned long curMillis;
unsigned long previousMillis;

long int positions[2];
int position1;
int position2;

long int count= 0;

char messageFromPC[buffSize] = {0};
float p1=0;
float p2=0;
float p3=0;
float p4=0;



// Define pin connections
const int dirPin1 = 4;
const int stepPin1 = 3;
const int dirPin2 = 6;
const int stepPin2 = 5;

int pos = 0;
// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper1(motorInterfaceType, stepPin1, dirPin1);
AccelStepper myStepper2(motorInterfaceType, stepPin2, dirPin2);
Servo myservo;

MultiStepper steppers;


void setup() {
  // put your setup code here, to run once:
  myStepper1.setMaxSpeed(100);
  myStepper1.setAcceleration(20);
  Serial.begin(115200);
  
  //myStepper.moveTo(200);
  myStepper1.setSpeed(120);
  myStepper1.setCurrentPosition(0);
  
  myStepper2.setMaxSpeed(100);
  myStepper2.setAcceleration(20);
 
  
  //myStepper.moveTo(200);
  myStepper2.setSpeed(60);
  myStepper2.setCurrentPosition(0);
  pinMode(dirPin1,OUTPUT);
  pinMode(stepPin1,OUTPUT);
  pinMode(dirPin2,OUTPUT);
  pinMode(stepPin2,OUTPUT);

  steppers.addStepper(myStepper1);
  steppers.addStepper(myStepper2);

  myservo.attach(9); 
  myservo.write(0); 
  
  Serial.print("<Arduino is ready>");
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  
  getDataFromPC();
  WhichMotion();
  replyToPC();

}


void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      Execute = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  p1 = atof(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  p2 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  p3 = atof(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  p4 = atof(strtokIndx);  
}


void replyToPC() {

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<END");
    Serial.print(messageFromPC);
    Serial.print(" p1 ");
    Serial.print(p1);
    Serial.print(" p2 ");
    Serial.print(p2);
    Serial.print(" pos1 ");
    Serial.print(position1);
    Serial.print(" pos2 ");
    Serial.print(position2);
    
    Serial.println(">");
    
    /*Serial.print("p3");
    Serial.print(p3);
    Serial.print("p4");
    Serial.print(p4);
    Serial.print("Positon1");
    Serial.print(position1);
    Serial.print("Positon2");
    Serial.print(position2);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");
    Serial.print("POS");
    Serial.print(CurrentPos);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");*/
  }
}


void WhichMotion() {

   // this illustrates using different inputs to call different functions
  if (strcmp(messageFromPC, "M1") == 0) {
     OneMotor();
     }

  if (strcmp(messageFromPC, "MM") == 0) {
     MultiMotor();
     }

   
   
}


void OneMotor(){

 
  
  //int angle = (200*p1)/360;
  
 /*myStepper.moveTo(angle);
  
  while(myStepper.distanceToGo()!=0) 
    {
      myStepper.run();
    }

  myStepper.move(45);
  myStepper.run();
  int y = myStepper.currentPosition();
  Serial.println(y);*/

  
  
 
  
}

void MultiMotor()
{
  long positions[2];
  positions[0] = 2*((400*p1)/360) ; // Running stepper with 1:2 gear ration and 1/2 step angle
  positions[1] = (400*p2)/360;

  myservo.write(p3);
  steppers.moveTo(positions);
  steppers.runSpeedToPosition();
  

  
  position1 = myStepper1.currentPosition();
  position2 = myStepper2.currentPosition();
  //Serial.println(position1);
  //Serial.println(position2);

  
}
