#include <SPI.h>
#include <LoRa.h>

// Pinos definidos especificamente para o ESP32 Heltec com LoRa
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Endereço do transmissor
#define SENDER_ADDRESS 42

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Configura os pinos do LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Inicializa o módulo LoRa na frequência desejada
  if (!LoRa.begin(915E6)) { // Altere a frequência conforme necessário
    Serial.println("Erro ao iniciar o módulo LoRa!");
    while (1);
  }

  Serial.println("Módulo LoRa iniciado com sucesso!");
}

void loop() {
  // Iniciar pacote de transmissão
  LoRa.beginPacket();

  // Enviar endereço do transmissor (42)
  LoRa.write(SENDER_ADDRESS);

  // Enviar a mensagem "Olá"
  LoRa.print("Olá");

  // Finalizar o pacote
  LoRa.endPacket();

  // Imprimir no Serial Monitor para depuração
  Serial.println("Mensagem enviada: Olá");

  // Aguardar um pouco antes de enviar a próxima mensagem
  delay(1000); // Aguardar 1 segundo
}
