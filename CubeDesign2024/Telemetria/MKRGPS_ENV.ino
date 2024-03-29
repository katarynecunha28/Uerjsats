#include <LoRa.h>
#include <Arduino_MKRENV.h>
#include <Arduino_MKRGPS.h>
#include <GPS.h>
#include <MKRENV.h>

// Endereço 
int meuEndereco = 42;

// Contador de pacotes
int counter = 0;

float latitude;
float longitude;
int satellites;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!LoRa.begin(915E6)) {
    Serial.println("Erro ao iniciar o módulo LoRa!");
    while (1);
  }
  Serial.println("Módulo LoRa iniciado com sucesso!");

  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }
  

  if (!GPS.begin()) {
    Serial.println("Failed to initialize GPS!");
    while (1);
  }

}

void loop() {
  Serial.println("Enviando mensagem...");

  // Ler todos os valores dos sensores
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();

  // Imprimir cada um dos valores dos sensores
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");

  Serial.print("Humidity    = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Pressure    = ");
  Serial.print(pressure);
  Serial.println(" kPa");

  Serial.print("Illuminance = ");
  Serial.print(illuminance);
  Serial.println(" lx");

  // read GPS values
  latitude   = GPS.latitude();
  longitude  = GPS.longitude();
  satellites = GPS.satellites();

  // print GPS values
  Serial.println();
  Serial.print("Location: ");
  Serial.print(latitude, 7);
  Serial.print(", ");
  Serial.println(longitude, 7);

  Serial.print("Number of satellites: ");
  Serial.println(satellites);

  Serial.println();


  // Construir a string para enviar via LoRa
  char stemp[120]; 
  sprintf(stemp, "%d, %f, %f, %f, %f, %f, %f, %d, %d", meuEndereco, temperature, humidity, pressure, illuminance, float(latitude)/1000000, float(longitude)/1000000, satellites, counter);

  // Enviar a mensagem via LoRa
  Serial.print("Sending packet: ");
  Serial.println(counter);
  LoRa.beginPacket();
  LoRa.print(stemp);
  LoRa.endPacket();

  // Imprimir a mensagem enviada
  Serial.println(stemp);
  Serial.println("Mensagem de teste do Arduino MKR WAN 1300");
  Serial.println("Mensagem enviada com sucesso!");

  // Incrementar o contador
  counter++;

  delay(5000);
}
