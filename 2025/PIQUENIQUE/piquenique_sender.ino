#include "DHT.h"
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configurações do DHT22
#define DHTPIN 12
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Configurações LoRa
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

int meuEndereco = 42;
char stemp[400];
int contadorPacotes = 0;

// OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// Setup
void setup()
{
  Serial.begin(9600);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LORA SENDER ");
  display.display();

  // Inicializa o DHT22
  dht.begin();

  // Inicializa o LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(908E6))
  {
    Serial.println("Falha ao inicializar o LoRa!");
    while (1);
  }

  Serial.println("LoRa Inicializado!");
  delay(1000);
}

// Loop principal
void loop()
{
  // Lê os valores do sensor DHT22
  float umidade = dht.readHumidity();
  float temperaturaDHT = dht.readTemperature();

  // Cria a mensagem com os dados
  sprintf(stemp, "%.2f, %.2f, Pacote: %d", temperaturaDHT, umidade, contadorPacotes);

  // Envia os dados via LoRa
  enviarSensores();
  
  // Atualize o display com o comando enviado
  display.clearDisplay();
  display.setCursor(0, 20);
  display.print("Enviando msg: ");
  display.setCursor(0, 30);
  display.print(stemp);
  display.display();


  delay(1500);
}

// Função para enviar dados via LoRa
void enviarSensores()
{
  LoRa.beginPacket();
  LoRa.write(meuEndereco);
  LoRa.print(stemp);
  LoRa.endPacket();

  contadorPacotes++;

  Serial.print("Mensagem enviada via LoRa: ");
  Serial.println(stemp);

  delay(1000);
}

