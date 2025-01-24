#include "DHT.h"

#define DHTPIN 12  
#define DHTTYPE DHT22   // DHT 22  
DHT dht(DHTPIN, DHTTYPE);

//LoRa 
#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Endereço único do rádio
int meuEndereco = 42;  

// Char para envio de dados dos sensores da placa de computador de bordo
char stemp[400]; 

// Contador de pacotes enviados
int contadorPacotes = 0; 

void setup() 
{
  Serial.begin(9600);

  // Inicializa o DHT22 (sensor de temperatura e umidade)
  dht.begin(); 

  // Seta os pinos SPI 
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Inicializa o rádio LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) 
  {
    Serial.println("Falha ao inicializar o LoRa!");
    while (1);
  }
  
  Serial.println("LoRa Inicializado!");
  delay(1000);
}

void loop() 
{
  // DHT22
  float umidade = dht.readHumidity();           // lê a umidade 
  float temperaturaDHT = dht.readTemperature(); // lê a temperatura

  // Cria o pacote com temperatura, umidade e contador
  sprintf(stemp, "%.2f, %.2f, Pacote: %d", temperaturaDHT, umidade, contadorPacotes); 

  enviarSensores(); // Envia os dados do computador de bordo via LoRa
  
  delay(1500);
}

void enviarSensores() 
{
  // Envia os dados dos sensores da placa do computador de bordo para a estação base
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Envia o endereço único para identificação
  LoRa.print(stemp);       // Envia a mensagem com os dados e o contador de pacotes
  LoRa.endPacket();

  // Incrementa o contador de pacotes
  contadorPacotes++;

  // Imprime os dados enviados no Serial Monitor
  Serial.print("Mensagem enviada via LoRa: ");
  Serial.println(stemp);

  delay(1000);
}
