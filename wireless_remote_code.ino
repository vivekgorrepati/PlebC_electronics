const int j1XPin = A0;
const int j1YPin = A1;
const int j2XPin = A2;
const int j2YPin = A3;
const int potPin = A4;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int potValue = analogRead(potPin);
  int x1Value = analogRead(j1XPin);
  int y1Value = analogRead(j1YPin);
  int x2Value = analogRead(j2XPin);
  int y2Value = analogRead(j2YPin);

  String output = "";

  // Add potValue to the output
  output += "#$" + String(potValue);

  // Check and print x1 condition
  if (x1Value > 900) {
    output += "$x1";
  } else if (x1Value < 200) {
    output += "$-x1";
  } else {
    output += "$0";
  }

  // Check and print y1 condition
  if (y1Value > 900) {
    output += "$y1";
  } else if (y1Value < 200) {
    output += "$-y1";
  } else {
    output += "$0";
  }

  // Check and print x2 condition
  if (x2Value > 900) {
    output += "$x2";
  } else if (x2Value < 200) {
    output += "$-x2";
  } else {
    output += "$0";
  }

  // Check and print y2 condition
  if (y2Value > 900) {
    output += "$y2";
  } else if (y2Value < 200) {
    output += "$-y2";
  } else {
    output += "$0";
     output += "*";
  }

  Serial.println(output); // Print the formatted output
}