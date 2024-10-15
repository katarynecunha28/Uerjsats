#include <Wire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>
#include <EEPROM.h> // Inclua a biblioteca EEPROM

// Definir pinos dos sensores DS18B20
#define DS18B20_1_PIN 2
#define DS18B20_2_PIN 3
#define DS18B20_3_PIN 4

// Inicializa o OneWire e DallasTemperature para cada sensor em pinos diferentes
OneWire oneWire1(DS18B20_1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(DS18B20_2_PIN);
DallasTemperature sensor2(&oneWire2);

OneWire oneWire3(DS18B20_3_PIN);
DallasTemperature sensor3(&oneWire3);

Adafruit_INA219 ina219;

unsigned long tempoInicial = 0;
bool comandoRecebido = false;

struct dadosVoo {
    float busVoltage;
    float current_mA;
    float tempbateria1;
    float tempbateria2;
    float tempbateria3;
};

const int tamanhoDadosVoo = sizeof(dadosVoo);
const int maxDados = 136; // Total de dados que você pode armazenar em 1024 bytes de EEPROM (Arduino Nano tem 1024 bytes de EEPROM)
int i = 0;

unsigned long tempoAnterior = 0;
const unsigned long intervalo = 30000; // 30 segundos

void setup() {
  Serial.begin(9600);

  // Inicializa os sensores DS18B20
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();

  // Inicializa o INA219
  if (!ina219.begin()) {
    Serial.println("Falha ao encontrar INA219!");
    while (1) { delay(10); }
  } 

  // Calibra o INA219 para leituras mais precisas (padrão para 32 V, 2A)
  ina219.setCalibration_32V_2A();
}

void loop() 
{
  receberComandoDoMaster(0); // Recebe os comandos destinados ao Suprimento de Energia
  delay(1000);

  unsigned long tempoAtual = millis();
  
  // Ler a tensão do barramento em volts
  float busVoltage = ina219.getBusVoltage_V();
  // Ler a corrente em miliamperes
  float current_mA = ina219.getCurrent_mA();

  // Lê a Temperatura dos sensores
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  sensor3.requestTemperatures();

  float tempbateria1 = sensor1.getTempCByIndex(0);
  float tempbateria2 = sensor2.getTempCByIndex(0);
  float tempbateria3 = sensor3.getTempCByIndex(0);
  
  // Verifica se 30 segundos se passaram
  if (tempoAtual - tempoAnterior >= intervalo) 
  {
      tempoAnterior = tempoAtual; // Atualiza o tempo anterior

      // Executa a gravação na EEPROM
      if (i < maxDados) 
      {
          dadosVoo voo;

          voo.busVoltage = busVoltage;
          voo.current_mA = current_mA;
          voo.tempbateria1 = tempbateria1;
          voo.tempbateria2 = tempbateria2;
          voo.tempbateria3 = tempbateria3;
                
          EEPROM.put(tamanhoDadosVoo * i, voo);
          i++;
      } 
      else 
      {
          // Se já gravou todos os dados, encerra o loop
          while (true) 
          {
            // Fica parado quando os dados forem completados
          }
      }
  }

  if (comandoRecebido) {
    enviarDadosParaMaster( String(busVoltage) + " " + String(current_mA) + " " + String(tempbateria1) + " " + String(tempbateria2) + " "+ String(tempbateria3), 0); // Envia dados para o Master
    comandoRecebido = false; // Reinicia a flag de comando recebido
    delay(2000);
  }

  delay(1000); // Aguarda 1 segundo antes de ler novamente
}

bool receberComandoDoMaster(int endereco) {
  if (Serial.available() > 0) {  //Lê a serial e vê se tem algum comando
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco) { //Se o endereço recebido for igual a 0, ele printa os dados recebidos do master
      String dados = dadosRecebidos.substring(2);
      comandoRecebido = true; //flag que define que um comando foi recebido
      tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 100ms
    }
  }
  return comandoRecebido;
}

void enviarDadosParaMaster(String dados, int endereco) {
  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(": ");
  Serial.println(dados);
}

