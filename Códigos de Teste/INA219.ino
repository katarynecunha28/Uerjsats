#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Hello!");

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
  } else {
    ina219.setCalibration_32V_2A();
    Serial.println("Measuring voltage and current with INA219 ...");
  }
}

void loop() {
  if (ina219.begin()) {
    float shuntVoltage = ina219.getShuntVoltage_mV();
    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float loadVoltage = busVoltage + (shuntVoltage / 1000);
    
    Serial.print("Bus Voltage:   "); Serial.print(busVoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadVoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.println();
  }
  
  delay(1000);
}

