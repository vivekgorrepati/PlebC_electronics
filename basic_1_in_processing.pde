import processing.serial.*;
 Serial myPort;
String myText ="";

void setup(){
 size (300,300);
 
 myPort = new Serial(this, "COM9",115200);
 myPort.bufferUntil('\n');
 }
 
 void serialEvent(Serial myPort){
 myText = myPort.readStringUntil('\n');
 
 }
 void draw(){
 background (0,0,0);
 text (myText, 120,120);
 myText = "";
 
 if (keyPressed && (keyCode == DOWN)){
 myPort.write('a');
 }
 else
 {
   myPort.write('c');
 }
 if (keyPressed && (keyCode == UP)){
 myPort.write('b');
 }
 else
 {
   myPort.write('d');
 }
 }
