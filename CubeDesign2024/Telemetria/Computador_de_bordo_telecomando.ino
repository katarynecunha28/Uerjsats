#include <LoRa.h>
#include <Arduino_MKRENV.h>
#include <MKRENV.h>

#define BAND 915E6

int meuEndereco = 41;  // Endereço do receptor
int endereçoTransmissor = 42;

void setup() {
  Serial.begin(9600);
  


  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");

  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1);
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    int senderAddress = LoRa.read();
    if (senderAddress == endereçoTransmissor) {
      String comando = LoRa.readString();
      Serial.println("Comando recebido: " + comando);
      processarComando(comando);
    }
  }
}

void processarComando(String comando) {
  if (comando.equals("A")) {
    enviarDadosParaSlave("Dados para Slave Pro mini", 1);
  } else if (comando.equals("B")) {
    enviarDadosParaSlave("Dados para Slave Nano", 2);
  } else {
    // Comando não reconhecido
    enviarResposta("Comando não reconhecido.");
    return;
  }

  // Aguardar resposta dos slaves
  aguardarResposta();
}

void aguardarResposta() {
  unsigned long tempoInicial = millis();  // Salva o tempo inicial
  while (millis() - tempoInicial < 5000) {
    if (receberDadosDosSlaves()) {
      // Se receber uma resposta de algum dos slaves, sai do loop
      break;
    }
  }
}

bool receberDadosDosSlaves() {
  if (Serial1.available() > 0 || Serial.available() > 0) {
    String dadosRecebidos;
    if (Serial1.available() > 0) {
      dadosRecebidos = Serial1.readStringUntil('\n');
    } else {
      dadosRecebidos = Serial.readStringUntil('\n');
    }

    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    // Verifica o endereço do dispositivo e processa os dados recebidos de acordo
    switch (enderecoRecebido) {
      case 3:
        processarDados("Pro mini Slave", dadosRecebidos);
        return true;
      case 4:
        processarDados("Arduino Nano Slave", dadosRecebidos);
        return true;
      default:
        return false;  // Retorna false se o endereço não for reconhecido
    }
  }
  return false;  // Retorna false se não receber os dados
}

void processarDados(String nomeSlave, String dados) {
  Serial.println("Dados recebidos do " + nomeSlave + ": " + dados);
  // Aqui você pode processar os dados recebidos, se necessário
}

void enviarDadosParaSlave(String dados, int endereco) {
  Serial1.print(endereco);  
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);  
  Serial.print(":");
  Serial.println(dados);
}

void enviarResposta(String resposta) {
  
  LoRa.beginPacket();
  LoRa.write(endereçoTransmissor); //Envia para o transmissor de número 42
  LoRa.print(resposta);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + resposta);
}
