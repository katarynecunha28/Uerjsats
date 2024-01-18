#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

Servo meuServo;

#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
#define BAND 915E6
#define MY_ADDRESS 42  // Mesmo valor configurado no LoRa Sender

// OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

String LoRaData;

void setup() { 
  Serial.begin(115200);
  
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("LORA RECEIVER ");
  display.display();

  Serial.println("LoRa Receiver Test");
  
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.println("LoRa Initializing OK!");
  display.display();  

  meuServo.attach(23);  // O pino ao qual o servo está conectado (por exemplo, pino 5)
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    int senderAddress = LoRa.read();
    
    if (senderAddress == MY_ADDRESS) {
      Serial.print("Received packet from address ");
      Serial.println(senderAddress);
      
      while (LoRa.available()) {
        LoRaData = LoRa.readString();
        Serial.print("Received data: ");
        Serial.println(LoRaData);

        // Execute comandos do servo com base nos dados recebidos
        if (LoRaData == "A") {
          moverServo(0, 180);
          enviarResposta("Comando A reconhecido.");
        } else if (LoRaData == "B") {
          moverServo(180, 0);
          enviarResposta("Comando B reconhecido.");
        } else {
          enviarResposta("Comando não reconhecido.");
        }
      }

      int rssi = LoRa.packetRssi();
      Serial.print(" with RSSI ");    
      Serial.println(rssi);

      // Dsiplay information
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("LORA RECEIVER");
      display.setCursor(0,20);
      display.print("Received packet:");
      display.setCursor(0,30);
      display.print(LoRaData);
      display.setCursor(0,40);
      display.setCursor(30,40);
      display.display();
    } else {
      Serial.println("Ignoring packet from unknown address.");
    }
  }
}

void moverServo(int from, int to) {
  if (from < to) {
    for (int grau = from; grau <= to; grau++) {
      meuServo.write(grau);
      delay(15);
    }
  } else {
    for (int grau = from; grau >= to; grau--) {
      meuServo.write(grau);
      delay(15);
    }
  }
  delay(500);  // Aumentei o delay para garantir tempo suficiente para o servo completar o movimento
}

void enviarResposta(String resposta) {
  LoRa.beginPacket();
  LoRa.print(resposta);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + resposta);
}
