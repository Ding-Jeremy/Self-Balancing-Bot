#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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

  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.println(az);

  delay(20);
}