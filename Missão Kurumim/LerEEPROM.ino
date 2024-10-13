#include <EEPROM.h> // Inclua a biblioteca EEPROM

struct dadosVoo {
    float temperaturaDHT;
    float umidade;
    float pressao;
    float altitude;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float valorSensor;
    float latitude;
    float longitude;
    int sats;
};

const int tamanhoDadosVoo = sizeof(dadosVoo);
const int maxDados = 727; // Total de dados que você quer armazenar

void setup() {
    Serial.begin(9600);
    EEPROM.begin(4096); // Inicializa a EEPROM

    // Leitura dos dados da EEPROM
    for (int i = 0; i < maxDados; i++) {
        dadosVoo voo;
        EEPROM.get(tamanhoDadosVoo * i, voo); // Lê os dados da EEPROM

        // Verifica se os valores lidos são válidos
        if (voo.temperaturaDHT != 0 || voo.umidade != 0 || voo.pressao != 0 || 
            voo.altitude != 0 || voo.accelX != 0 || voo.accelY != 0 || 
            voo.accelZ != 0 || voo.gyroX != 0 || voo.gyroY != 0 || 
            voo.gyroZ != 0 || voo.valorSensor != 0 || 
            voo.latitude != 0 || voo.longitude != 0 || 
            voo.sats != 0) { // Verifica se pelo menos um dos valores é diferente de zero
            Serial.print("Registro ");
            Serial.print(i);
            Serial.print(": Temperatura: ");
            Serial.print(voo.temperaturaDHT);
            Serial.print(", Umidade: ");
            Serial.print(voo.umidade);
            Serial.print(", Pressão: ");
            Serial.print(voo.pressao);
            Serial.print(", Altitude: ");
            Serial.print(voo.altitude);
            Serial.print(", Acel X: ");
            Serial.print(voo.accelX);
            Serial.print(", Acel Y: ");
            Serial.print(voo.accelY);
            Serial.print(", Acel Z: ");
            Serial.print(voo.accelZ);
            Serial.print(", Giro X: ");
            Serial.print(voo.gyroX);
            Serial.print(", Giro Y: ");
            Serial.print(voo.gyroY);
            Serial.print(", Giro Z: ");
            Serial.print(voo.gyroZ);
            Serial.print(", Valor do Sensor: ");
            Serial.print(voo.valorSensor);
            Serial.print(", Latitude: ");
            Serial.print(voo.latitude);
            Serial.print(", Longitude: ");
            Serial.print(voo.longitude);
            Serial.print(", Sats: ");
            Serial.println(voo.sats);
        } else {
            Serial.print("Registro ");
            Serial.print(i);
            Serial.println(": Dados vazios ou zeros.");
        }
    }
}

void loop() {
    // O loop pode ser deixado vazio se não houver necessidade de repetir a leitura
}
