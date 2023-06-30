#include <Wire.h>
#include <MPU9150.h>

MPU9150 mpu;

int16_t ax, ay, az;  // Accelerometer readings
int16_t gx, gy, gz;  // Gyroscope readings

void setup() {
   
  Serial.begin(115200);
  Serial.println("start");
  Wire.begin();  // Initialize I2C bus
  
  
  // Initialize MPU-9150
  mpu.initialize();
  
  // Check if MPU-9150 connection was successful
  if (mpu.testConnection()) {
    Serial.println("MPU-9150 connection successful!");
  } else {
    Serial.println("MPU-9150 connection failed!");
    while (1);
  }
  
  // Configure MPU-9150 settings
  mpu.setFullScaleAccelRange(MPU9150_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU9150_GYRO_FS_250);
  mpu.setDLPFMode(MPU9150_DLPF_BW_42);
}

void loop() {
  // Read sensor data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print the sensor readings
  Serial.print("Accelerometer (g): ");
  Serial.print("X = "); Serial.print(ax); Serial.print(", ");
  Serial.print("Y = "); Serial.print(ay); Serial.print(", ");
  Serial.print("Z = "); Serial.println(az);

  Serial.print("Gyroscope (°/s): ");
  Serial.print("X = "); Serial.print(gx); Serial.print(", ");
  Serial.print("Y = "); Serial.print(gy); Serial.print(", ");
  Serial.print("Z = "); Serial.println(gz);

  delay(1000);  // Delay for 1 second
}
