unsigned long tempoInicial;

#include <LoRa.h>
#include <Arduino_MKRENV.h>
#include <MKRENV.h>

// Endereço 
int meuEndereco = 42;

// Contador de pacotes
int counter = 0;

void setup() {
  Serial.begin(9600);  // Serial para debug e Slave Uno
  Serial1.begin(9600);

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
}

void loop() {
  Serial.println("Enviando mensagem...");

  // Ler todos os valores dos sensores
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();

  Serial.println();


  // Construir a string para enviar via LoRa
  char stemp[120]; 
  sprintf(stemp, "%d, %f, %f, %f, %f, %d", meuEndereco, temperature, humidity, pressure, illuminance, counter);

  delay(5000);

  if (Serial1.available() > 0 || Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    if (comando == "enviar1") { //Switch case 
      enviarDadosParaSlave("Dados para Slave Pro mini", 1); //Mudar pra uma Letra 
      aguardarResposta();
    } else if (comando == "enviar2") {
      enviarDadosParaSlave("Dados para Slave Nano", 2);
      aguardarResposta();
    }
  }

    // Enviar a mensagem via LoRa
  Serial.print("Sending packet: ");
  Serial.println(counter);
  LoRa.beginPacket();
  LoRa.print(stemp);

  LoRa.endPacket();

    // Incrementar o contador
  counter++;

 // Imprimir a mensagem enviada
  Serial.println(stemp);
  Serial.println("Mensagem de teste do Arduino MKR WAN 1300");
  Serial.println("Mensagem enviada com sucesso!");

}

void aguardarResposta() {
  tempoInicial = millis();  // Salva o tempo inicial
  while (millis() - tempoInicial < 5000) {
    if (receberDadosDoSlaves()) {
      // Se receber uma resposta de algum dos slaves, sai do loop
      break;
    }
  }
}

bool receberDadosDoSlaves() {
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
        processarDados("Pro mini Slave", dadosRecebidos.substring(1));
        return true;
      case 4:
        processarDados("Arduino Nano Slave", dadosRecebidos.substring(1));
        return true;
      default:
        return false;  // Retorna false se o endereço não for reconhecido
    }
  }
  return false;  // Retorna false se não receber os dados
}

void processarDados(String nomeSlave, String dados) {
  Serial.println("Dados recebidos do " + nomeSlave + ": " + dados);
  Serial1.println("Dados recebidos do " + nomeSlave + ": " + dados);
  LoRa.beginPacket();
  LoRa.print(dados);
  LoRa.endPacket();
  
}

void enviarDadosParaSlave(String dados, int endereco) {
  Serial1.print(endereco);  // Identificador de destino
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(":");
  Serial.println(dados);
}
