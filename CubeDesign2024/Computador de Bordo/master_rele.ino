
unsigned long tempoInicial;

#include <LoRa.h>
#include <Arduino_MKRENV.h>
#include <MKRENV.h>



int meuEndereco = 42;  // Endereço do receptor
int pinRele = 7;

char stemp[120]; 

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(pinRele, OUTPUT);

  while(!Serial);
  
  if (!LoRa.begin(433E6)) {
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
    if (senderAddress == meuEndereco) {
      String comando = LoRa.readString();
      Serial.println("Comando recebido: " + comando);
      processarComando(comando);
    }
  }
  receberDadosDosSlaves();

  // Ler todos os valores dos sensores
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();


  // Construir a string para enviar via LoRa

  sprintf(stemp, "%d, %f, %f, %f, %f", meuEndereco, temperature, humidity, pressure, illuminance);

}

void processarComando(String comando) {
  //Serial.println("foi");
  if (comando.equals("A")) {
    enviarDadosParaSlave("Ligar motor", 1);

    delay(100);
  } else if (comando.equals("B")) {
    enviarDadosParaSlave("Dados Suprimento de Energia", 2);
    delay(100);
  } else if (comando.equals("C")) {
    enviarSensores();
    delay(100);
  }
    else if (comando.equals("D")){
      enviarComandoParaPython("Dados Portenta", "5");
    }
    else if (comando.equals("E")){
      digitalWrite(pinRele, HIGH);
      delay(100);
      digitalWrite(pinRele, LOW);
      Serial.println("Relé acionado.");
      enviarResposta("Relé acionado.");
      delay(100);
    }
    else if (comando.equals("F")){
      enviarDadosParaSlave("Mudar angulo", 1);
    }
    else {
    // Comando não reconhecido
    enviarResposta("Comando não reconhecido.");
    return;
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
        processarDados("Controle de Atitude", dadosRecebidos.substring(1));
        return true;
      case 4:
        processarDados("Suprimento", dadosRecebidos.substring(1));
        return true;

      case 6:
        processarDados("Portenta", dadosRecebidos.substring(1));
        return true;

      default:
        return false;  // Retorna false se o endereço não for reconhecido
    }
  }
  return false;  // Retorna false se não receber os dados
  Serial.println("nao recebi");
}

void processarDados(String nomeSlave, String dados) {
  Serial1.println("Dados recebidos do " + nomeSlave + ": " + dados);
  Serial.println("Dados recebidos do " + nomeSlave + ": " + dados);
  
  enviarResposta(dados);
}

void enviarDadosParaSlave(String dados, int endereco) {
  Serial1.print(endereco);  // Identificador de destino
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(":");
  Serial.println(dados);
}

void enviarResposta(String resposta) {
  // Envia a resposta de volta para o transmissor
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Endereço do receptor
  LoRa.print(resposta);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + resposta);
}

void enviarSensores() {
  // Envia a resposta de volta para o transmissor
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Endereço do receptor
  LoRa.print(stemp);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + String(stemp));
}

void enviarComandoParaPython(String endereco, String dados){
  Serial1.print(endereco);  // Identificador de destino
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(":");
  Serial.println(dados);
}

void receberDadosDoPython() {
  if (Serial1.available() > 0) {
    String dadosRecebidos = Serial1.readStringUntil('\n');
   

    // Processar os dados recebidos do Python conforme necessário
    Serial1.print("Dados recebidos do Portenta:");
    Serial.print("Dados recebidos do Python: ");
    Serial1.println(dadosRecebidos);
    Serial.println(dadosRecebidos);
    enviarResposta(dadosRecebidos);

  }
}
