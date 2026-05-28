#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

float roll, pitch;
float angle = 0;
unsigned long prevTime = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (mpu.testConnection())
  {
    Serial.println("MPU6050 connection successful");
  }
  else
  {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
}

void loop()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Compute accelerometer angles
  roll = atan2((float)ay, (float)az) * 180.0 / PI;
  pitch = atan2(-(float)ax, sqrt((float)ay * (float)ay + (float)az * (float)az)) * 180.0 / PI;

  // Time step
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  // Gyro rate (°/s)
  float gyroXrate = (float)gx / 131.0;

  // Complementary filter
  angle = 0.98 * (angle + gyroXrate * dt) + 0.02 * pitch;

  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(angle);

  delay(5);
}