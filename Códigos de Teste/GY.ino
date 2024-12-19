#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>

#define ADXL345_ADDRESS (0x53)
#define ITG3200_ADDRESS (0x69)
#define HMC5883L_ADDRESS (0x1E)
#define BMP085_ADDRESS (0x77)

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(ADXL345_ADDRESS);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(ITG3200_ADDRESS);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(HMC5883L_ADDRESS);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(BMP085_ADDRESS);

void setup() {
  Serial.begin(115200);

  if (!accel.begin()) {
    Serial.println("Acelerômetro não encontrado. Verifique as conexões!");
    while (1);
  }

  if (!gyro.begin()) {
    Serial.println("Giroscópio não encontrado. Verifique as conexões!");
    while (1);
  }

  if (!mag.begin()) {
    Serial.println("Sensor de campo magnético não encontrado. Verifique as conexões!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println("Sensor BMP085 não encontrado. Verifique as conexões!");
    while (1);
  }

  Serial.println("------------------------------------");
  Serial.println("GY-80 Módulo de Sensores");
  Serial.println("------------------------------------");
}

void loop() {
  sensors_event_t accelEvent, gyroEvent, magEvent, bmpEvent;
  
  accel.getEvent(&accelEvent);
  gyro.getEvent(&gyroEvent);
  mag.getEvent(&magEvent);
  bmp.getEvent(&bmpEvent);

  Serial.print("Acelerômetro (m/s^2): ");
  Serial.print("X = "); Serial.print(accelEvent.acceleration.x);
  Serial.print(" | Y = "); Serial.print(accelEvent.acceleration.y);
  Serial.print(" | Z = "); Serial.println(accelEvent.acceleration.z);

  Serial.print("Giroscópio (rad/s): ");
  Serial.print("X = "); Serial.print(gyroEvent.gyro.x);
  Serial.print(" | Y = "); Serial.print(gyroEvent.gyro.y);
  Serial.print(" | Z = "); Serial.println(gyroEvent.gyro.z);

  Serial.print("Campo Magnético (uT): ");
  Serial.print("X = "); Serial.print(magEvent.magnetic.x);
  Serial.print(" | Y = "); Serial.print(magEvent.magnetic.y);
  Serial.print(" | Z = "); Serial.println(magEvent.magnetic.z);

  Serial.print("Pressão (hPa): ");
  Serial.println(bmpEvent.pressure);

  Serial.println("------------------------------------");
  delay(1000);
}



