#include <EEPROM.h>

void setup() {
  Serial.begin(9600);
  EEPROM.begin(4096); // Inicializa a EEPROM (tamanho da EEPROM)

  // Apaga toda a EEPROM, preenchendo com 0
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0); // Grava 0 em cada posição de memória
  }

  EEPROM.commit(); // Salva as alterações feitas na EEPROM

  Serial.println("EEPROM apagada com sucesso.");
}

void loop() {
  // O loop é mantido vazio
}
