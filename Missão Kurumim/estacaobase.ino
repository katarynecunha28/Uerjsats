#include <SPI.h>
#include <LoRa.h>

// Pinos definidos especificamente para o ESP32 Heltec com LoRa
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Endereço que o receptor deve aceitar
#define RECEIVER_ADDRESS 42

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
  // Verifica se há algum pacote recebido
  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    // Lê o endereço de destino do pacote
    int destinationAddress = LoRa.read();

    // Verifica se o endereço de destino é o correto
    if (destinationAddress == RECEIVER_ADDRESS) {
      // Lê a mensagem
      String receivedMessage = "";
      while (LoRa.available()) {
        receivedMessage += (char)LoRa.read();
      }

      // Imprime a mensagem recebida no Serial Monitor
      Serial.print("Mensagem recebida do endereço ");
      Serial.print(destinationAddress);
      Serial.print(": ");
      Serial.println(receivedMessage);
    } else {
      // Ignorar pacotes com endereços de destino diferentes
      Serial.println("Pacote ignorado devido ao endereço de destino incorreto");
    }
  }
}
