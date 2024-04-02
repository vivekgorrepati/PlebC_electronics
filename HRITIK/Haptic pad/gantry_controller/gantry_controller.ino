String receivedString;
String gantryValues;
String forceValue;
String quaternionValues;

// Variables to store the angles for each motor
float angle_1 = 0;
String value_2;
String value_3;
String value_4;
String value_5;

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  if (Serial.available() > 0) {
    receivedString = Serial.readString(); // Read the incoming data as a String

    // Extract gantry values from the received string
    int gantryIndex = receivedString.indexOf("gantry_value");
    if (gantryIndex != -1) {
      gantryValues = receivedString.substring(gantryIndex + 15); // Extract substring after "gantry_value : "
      gantryValues.trim(); // Remove leading and trailing whitespaces
    }
    // Extract force value from the received string
    int forceIndex = receivedString.indexOf("Force");
    if (forceIndex != -1) {
      forceValue = receivedString.substring(forceIndex + 8); // Extract substring after "Force : "
      forceValue.trim(); // Remove leading and trailing whitespaces
    }

    // Extract quaternion values from the received string
    int quatIndex = receivedString.indexOf("localSystem_value");
    if (quatIndex != -1) {
      quaternionValues = receivedString.substring(quatIndex + 20); // Extract substring after "localSystem_value : "
      quaternionValues.trim(); // Remove leading and trailing whitespaces
    }
      
      // Printing the extracted values for this iteration
      //Serial.print("Gantry Values: ");
      Serial.println(gantryValues);
      //Serial.print("Force: ");
      Serial.println(forceValue);
      //Serial.print("Quaternion values: ");
      Serial.println(quaternionValues);

      // Split the gantryValues string into angle values
      char *token;
      char *valueStr;
      char receivedChars[gantryValues.length() + 1];
      strcpy(receivedChars, gantryValues.c_str());

      // Process the values
      token = strtok(receivedChars, "$");
      int valueIndex = 1;
      while (token != NULL && valueIndex <= 5)
      {
          valueStr = token;
          if (valueIndex == 1) {
              angle_1 = atof(valueStr);
          } else {
              switch (valueIndex) {
                  case 2:
                      value_2 = valueStr;
                      break;
                  case 3:
                      value_3 = valueStr;
                      break;
                  case 4:
                      value_4 = valueStr;
                      break;
                  case 5:
                      value_5 = valueStr;
                      break;
              }
          }

          token = strtok(NULL, "$");
          valueIndex++;
      }

      // Print values for debugging
      Serial.println(angle_1);
      Serial.println(value_2);
      Serial.println(value_3);
      Serial.println(value_4);
      Serial.println(value_5);
      
      // Clearing the variables for the next iteration
      receivedString = "";
      gantryValues = "";
      forceValue = "";
      quaternionValues = "";
    }
  }

