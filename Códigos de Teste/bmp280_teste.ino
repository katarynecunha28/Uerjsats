#include <Wire.h>
#include <Adafruit_BMP280.h>

#define BMP_SDA A4  // Pino SDA do BMP280 conectado ao pino 21 do ESP32
#define BMP_SCL A5  // Pino SCL do BMP280 conectado ao pino 22 do ESP32

#define SerialMonitor Serial

Adafruit_BMP280 bmp; // Inicialização do objeto BMP280


void setup() {
  SerialMonitor.begin(9600);

  // Inicialização do sensor BMP280
  if (!bmp.begin()) {
    Serial.println("Não foi possível encontrar o sensor BMP280. Verifique as conexões!");
    while (1);
  }
}

void loop() {
  // Leitura da pressão e temperatura
  float pressao = bmp.readPressure() / 100.0F; // Convertendo para hPa
  float temperatura = bmp.readTemperature();

  // Exibição dos valores no console serial
  Serial.print("Pressão: ");
  Serial.print(pressao);
  Serial.print(" hPa\t");
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");

  // Aguarde um intervalo de tempo antes da próxima leitura
  delay(1000);
}
