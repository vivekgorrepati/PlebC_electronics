// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2015-08-03 - added Arduino 1.6 support
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#define MPU6050_INTERRUPT_FIFO_OFLOW_BIT 4
#define MPU6050_INTMODE_FIFO_BUFFER_OVERFLOW 0x10
int packetSize = 0;  // Declare and initialize packetSize variable
int fifoCount = 0;  // Declare and initialize fifoCount variable
Quaternion q = Quaternion(0, 0, 0, 0); // Initialize with default values
float gravity;// Initialize with default values
float ypr; // Initialize with default values




#define MPU6050_ADDRESS 0x68 // I2C address of the MPU6050
uint8_t fifoBuffer[1024];

MPU6050 mpu;
volatile bool mpuIntStatus = false;

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    // Initialize MPU6050
    mpu.initialize();

    // Verify connection
    Serial.println("Testing MPU6050 connection...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Load and configure the DMP
    Serial.println("Initializing DMP...");
    uint8_t devStatus = mpu.dmpInitialize();

    // Calibrate gyro and accelerometer, load biases in bias registers
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // Set DMP ready flag
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);

        // Enable interrupt detection
        attachInterrupt(digitalPinToInterrupt(GPIO_PIN_13), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our desired DMP interrupt behavior (if any)
        mpuIntStatus |= MPU6050_INTERRUPT_FIFO_OFLOW_BIT;

        // Enable DMP interrupt
        mpu.setInterruptMode(MPU6050_INTMODE_FIFO_BUFFER_OVERFLOW);
        mpuIntStatus = mpu.getIntStatus();

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void loop()
{
    // Wait for MPU interrupt or extra packets available
    if (!mpuInterrupt && fifoCount < packetSize)
    {
        return;
    }

    // Reset interrupt flag and get MPU data
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
   // mpu.setIntEnabled(MPU6050_INTERRUPT_DMP_INT_BIT);
   

    // Check for overflow and reset if necessary
    if ((mpuIntStatus & MPU6050_INTERRUPT_FIFO_OFLOW_BIT) || fifoCount >= 1024)
    {
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");

    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus )
    {
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }

        // Read packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Process packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Display Euler angles
        Serial.print("Yaw: ");
        Serial.print(ypr[0] * 180 / M_PI);
        Serial.print(" Pitch: ");
        Serial.print(ypr[1] * 180 / M_PI);
        Serial.print(" Roll: ");
        Serial.println(ypr[2] * 180 / M_PI);
    }
}

