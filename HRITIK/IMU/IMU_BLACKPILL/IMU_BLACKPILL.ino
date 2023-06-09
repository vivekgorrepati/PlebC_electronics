// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelGyroMag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro/mag measurements from device
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro/mag x/y/z values
//  Serial.print("a/g/m:\t");
//  Serial.print(ax); Serial.print("\t");
//  Serial.print(ay); Serial.print("\t");
//  Serial.print(az); Serial.print("\t");
//  Serial.print(gx); Serial.print("\t");
//  Serial.print(gy); Serial.print("\t");
//  Serial.print(gz); Serial.print("\t");
    //Serial.print(int(mx)*int(mx)); Serial.print("\t");
    //Serial.print(int(my)*int(my)); Serial.print("\t");
    //Serial.print(int(mz)*int(mz)); Serial.print("\t | ");

    //const float N = 256;
    //float mag = mx*mx/N + my*my/N + mz*mz/N;

    //Serial.print(mag); Serial.print("\t");
    //for (int i=0; i<mag; i++)
    //    Serial.print("*");


    //Sensitivity Scale Factor:
    //| MIN   | TYP | MAX   | UNITS   |
    //| 0.285 | 0.3 | 0.315 | uT /LSB |
    //compass data in uT
    Serial.print(mx * 0.3f); Serial.print("\t");
    Serial.print(my * 0.3f); Serial.print("\t");
    Serial.print(mz * 0.3f); Serial.println("\t | uT");

    //Serial.print("\n");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(50);
}