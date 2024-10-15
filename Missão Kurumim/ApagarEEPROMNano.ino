#include <EEPROM.h>

void setup() {
  Serial.begin(9600);
  Serial.println("Apagando a EEPROM...");

  // Percorre toda a memória da EEPROM e grava 0 em cada byte
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }

  Serial.println("EEPROM apagada.");
}

void loop() {
  // Não há necessidade de nada no loop
}
