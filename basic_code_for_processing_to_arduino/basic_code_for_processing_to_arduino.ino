const int led = 13;
int ledState = 0;  
int buttonState = 0;     
const int buttonPin = 2; 
void setup(){
  pinMode(led, OUTPUT);
    pinMode(buttonPin, INPUT);
  Serial.begin(9600);
  }

  void loop(){
    
    if (Serial.available() > 0){
      buttonState = digitalRead(buttonPin);
      char ledState = Serial.read();
      if((ledState=='1')){
        digitalWrite(led,HIGH);
      }else{
        digitalWrite(led,LOW);
      }}}
  
