#include "DHT.h"

#define PINO_DHT 12

#define TIPO_DHT DHT22

DHT dht(PINO_DHT, TIPO_DHT);

void setup(){
  Serial.begin(9600);
  Serial.println("DHT22 teste!");

  dht.begin();
}

void loop(){
  delay(2000);
  float umidade = dht.readHumidity();
  float temperatura = dht.readTemperature();
  
  Serial.println("Umidade: ", umidade);
  Serial.println("Temperatura: ", temperatura);
}