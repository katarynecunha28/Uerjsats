#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

const int pinLM35 = A0; // Pino analógico ao qual o LM35 está conectado

unsigned long tempoInicial = 0;
bool comandoRecebido = false;

void setup() {
  Serial.begin(9600);
  ina219.begin();
}

void loop() {
  receberDadosDoMaster(2);  // Escuta mensagens destinadas ao Slave Uno
  delay(1000);
  Serial.println("apos delay");
  Serial.println(comandoRecebido);
  

  if (comandoRecebido) {
    if (millis() - tempoInicial >= 100) {
      float corrente = ina219.getCurrent_mA();
      float tensao = ina219.getBusVoltage_V();
      float temperatura = lerTemperatura(); // Ler temperatura do LM35
      String dados = "Corrente: " + String(corrente) + "mA, Tensao: " + String(tensao) + "V, Temperatura: " + String(temperatura) + "C";
      Serial.println(dados);
      enviarDadosParaMaster(dados, "4");  // Envia para o Master
      comandoRecebido = false; // Reinicia a flag de comando recebido
      Serial.println("if concluido");     
    }
  }
}

float lerTemperatura() {
  int valorADC = analogRead(pinLM35); // Lê o valor analógico do LM35
  float tensao = (valorADC / 1023.0) * 5.0; // Converte o valor para tensão (5V refere-se à tensão de referência do Arduino)
  float temperatura = (tensao - 0.5) * 100.0; // Converte a tensão para temperatura (0.5V refere-se a 0 graus Celsius)
  return temperatura;
}

void receberDadosDoMaster(int endereco) {
  if (Serial.available() > 0) {
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco) {
      String dados = dadosRecebidos.substring(2);
      Serial.println("Dados recebidos do Master: " + dados);
      comandoRecebido = true; // Define que um comando foi recebido
      tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 10 segundos
    }
  }
}

void enviarDadosParaMaster(String dados, int endereco) {
  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(": ");
  Serial.println(dados);
}
