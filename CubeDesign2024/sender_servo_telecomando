#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define BAND 915E6

// OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
  Serial.begin(115200);
  
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
  
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    enviarComando(comando);
  }
}

void enviarComando(String comando) {
  LoRa.beginPacket();
  LoRa.print(comando);
  LoRa.endPacket();
  
  Serial.println("Comando enviado: " + comando);

  // Atualize o display com o comando enviado
  display.clearDisplay();
  display.setCursor(0, 20);
  display.print("Enviando comando: ");
  display.setCursor(0, 30);
  display.print(comando);
  display.display();

  // Aguarde uma resposta do LoRa Receiver
  int timeout = 5000;  // Tempo limite em milissegundos
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String resposta = LoRa.readString();
      Serial.println("Resposta do Receiver: " + resposta);

      // Atualize o display com a resposta do Receiver
      display.clearDisplay();
      display.setCursor(0, 20);
      display.print("Resposta do Receiver: ");
      display.setCursor(0, 30);
      display.print(resposta);
      display.display();

      return;
    }
  }

  // Se não houver resposta dentro do tempo limite
  Serial.println("Timeout: Nenhuma resposta do Receiver.");
  display.clearDisplay();
  display.setCursor(0, 20);
  display.print("Timeout: Nenhuma resposta do Receiver.");
  display.display();
}
