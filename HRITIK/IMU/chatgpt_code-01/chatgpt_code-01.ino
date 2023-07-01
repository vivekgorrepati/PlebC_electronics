#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.println("working");
  Wire.begin();  // Initialize I2C bus
  Serial.begin(9600);

  Serial.println("working");

  // Initialize MPU-6050
  mpu.initialize();

  // Check if MPU-6050 connection was successful
  if (mpu.testConnection()) {
    Serial.println("MPU-6050 connection successful!");
  } else {
    Serial.println("MPU-6050 connection failed!");
    while (1);
  }

  // Configure MPU-6050 settings
  mpu.setFullScaleAccelRange(0); // Set accelerometer full-scale range to +/- 2g
  mpu.setFullScaleGyroRange(0);  // Set gyroscope full-scale range to +/- 250 degrees/s
}

void loop() {
  // Read sensor data
  int16_t ax, ay, az;  // Accelerometer readings
  int16_t gx, gy, gz;  // Gyroscope readings

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Print the sensor readings
  Serial.print("Accelerometer (g): ");
  Serial.print("X = "); Serial.print(ax / 16384.0); Serial.print(", ");
  Serial.print("Y = "); Serial.print(ay / 16384.0); Serial.print(", ");
  Serial.print("Z = "); Serial.println(az / 16384.0);

  Serial.print("Gyroscope (°/s): ");
  Serial.print("X = "); Serial.print(gx / 131.0); Serial.print(", ");
  Serial.print("Y = "); Serial.print(gy / 131.0); Serial.print(", ");
  Serial.print("Z = "); Serial.println(gz / 131.0);

  delay(1000);  // Delay for 1 second
}
