#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

unsigned long tempoInicial = 0;
bool comandoRecebido = false;


void setup() {
  Serial.begin(9600);

  //Inicializa o IN4219
  if (!ina219.begin()) {
    Serial.println("Falha ao encontrar IN4219!");
    while (1) {delay(10);}
  } 

  //Calibra o IN4219 para leituras mais precisas (padrão para 32 V, 2A)
  ina219.setCalibration_32V_2A();

}

void loop() {
  
  receberComandoDoMaster(2); // Recebe os comandos destinadas ao Suprimento de Energia
 
  delay(1000);
  
    // Ler a tensão do barramento em volts
    float busVoltage = ina219.getBusVoltage_V();
    // Ler a corrente em miliamperes
    float current_mA = ina219.getCurrent_mA();

    //Printa os valores da tensão do barramento e tensão na serial
    
    Serial.print("Tensão do Barramento:   "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Corrente:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.println();

  Serial.println("comando recebido: " + comandoRecebido);

  if (comandoRecebido) {
      
      enviarDadosParaMaster("Dados do Suprimento de Energia: " + String(busVoltage) + "V " + String(current_mA) + "mA ", 4); // Envia dados para o Master
      comandoRecebido = false; // Reinicia a flag de comando recebido
    }
  
  delay(1000); // Aguarda 1 segundo antes de ler novamente

  }

bool receberComandoDoMaster(int endereco) {
  if (Serial.available() > 0) {  //Lê a serial e vê se tem algum comando
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco) { //Se o endereço recebido for igual a 2, ele printa os dados recebidos do master
      String dados = dadosRecebidos.substring(2);
      Serial.println("Comando recebido do Master: " + dados);
      comandoRecebido = true; //flag que define que um comando foi recebido
      tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 100ms
    }
  }
}

void enviarDadosParaMaster(String dados, int endereco) {
  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(": ");
  Serial.println(dados);

}




