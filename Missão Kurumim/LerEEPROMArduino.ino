#include <EEPROM.h>

// Estrutura que corresponde ao formato de dados armazenados na EEPROM
struct dadosVoo {
    float busVoltage;
    float current_mA;
    float tempbateria1;
    float tempbateria2;
    float tempbateria3;
};

const int tamanhoDadosVoo = sizeof(dadosVoo); // Tamanho da estrutura
const int maxDados = 136; // Número máximo de registros que você gravou na EEPROM

void setup() {
  Serial.begin(9600);
  delay(1000);

  // Lê e exibe todos os dados armazenados na EEPROM
  for (int i = 0; i < maxDados; i++) {
    dadosVoo voo;
    EEPROM.get(tamanhoDadosVoo * i, voo);

    // Exibe os dados lidos da EEPROM no monitor serial
    Serial.print("Registro ");
    Serial.print(i + 1);
    Serial.println(":");
    
    Serial.print("  Tensão do Barramento: ");
    Serial.print(voo.busVoltage);
    Serial.println(" V");

    Serial.print("  Corrente: ");
    Serial.print(voo.current_mA);
    Serial.println(" mA");

    Serial.print("  Temp. Bateria 1: ");
    Serial.print(voo.tempbateria1);
    Serial.println(" °C");

    Serial.print("  Temp. Bateria 2: ");
    Serial.print(voo.tempbateria2);
    Serial.println(" °C");

    Serial.print("  Temp. Bateria 3: ");
    Serial.print(voo.tempbateria3);
    Serial.println(" °C");

    Serial.println("-------------------------");
    delay(500);  // Atraso para facilitar a leitura dos dados no monitor serial
  }
}

void loop() {
  // Não é necessário nada no loop para este exemplo
}

