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

String LoRaData;

void setup() { 
  Serial.begin(115200);
  


  Serial.println("LoRa Receiver Test");
  
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");

}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("RECEBI ALGO");
    int senderAddress = LoRa.read();
    

      Serial.print("Received packet from address ");
      Serial.println(senderAddress);
      
      while (LoRa.available()) {
        LoRaData = LoRa.readString();
        Serial.print("Received data: ");
        Serial.println(LoRaData);
      }

      int rssi = LoRa.packetRssi();
      Serial.print(" with RSSI ");    
      Serial.println(rssi);

  }
}
