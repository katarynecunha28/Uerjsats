#include <Wire.h>
#include <MPU6050.h>

// Crie uma instância do objeto MPU6050
MPU6050 mpu;

void setup() {
  Serial.begin(9600);

  // Inicialize o sensor MPU6050
  Wire.begin();
  mpu.initialize();
}

void loop() {
  // Leia os dados do MPU6050
 // mpu.update();

  // Obtenha os valores dos eixos X, Y e Z do giroscópioA
  int gyroX = mpu.getAngleX();
  int gyroY = mpu.getAngleY();
  int gyroZ = mpu.getAngleZ();

  // Obtenha os valores dos eixos X, Y e Z do acelerômetro
  int accX = mpu.getAccX();
  int accY = mpu.getAccY();
  int accZ = mpu.getAccZ();

  // Imprima os valores no Serial Monitor
  Serial.print("Gyro X: ");
  Serial.print(gyroX);
  Serial.print("\tGyro Y: ");
  Serial.print(gyroY);
  Serial.print("\tGyro Z: ");
  Serial.print(gyroZ);

  Serial.print("\tAcc X: ");
  Serial.print(accX);
  Serial.print("\tAcc Y: ");
  Serial.print(accY);
  Serial.print("\tAcc Z: ");
  Serial.println(accZ);

  // Aguarde um pouco antes de ler os próximos valores
  delay(1000);
}
