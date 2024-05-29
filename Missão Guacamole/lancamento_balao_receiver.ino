#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 23
#define LORA_DIO0 26

#define MY_ADDRESS 42 // Your receiver address

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize LoRa module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa initialization failed");
    while (1);
  }

  // Set receiver address
  LoRa.setSyncWord(0xF3); // Adjust sync word to match transmitter if necessary
  LoRa.setAddress(MY_ADDRESS); // Set receiver address
  LoRa.enableCrc(); // Enable CRC for packet validation
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet: ");

    // check packet header to ensure it's intended for this receiver
    if (LoRa.packetRfStyle() == 1 && LoRa.packetRfHeader() == MY_ADDRESS) {
      // read packet
      while (LoRa.available()) {
        Serial.print((char)LoRa.read());
      }

      // print RSSI of packet
      Serial.print(" with RSSI ");
      Serial.println(LoRa.packetRssi());
    } else {
      // Packet is not intended for this receiver
      Serial.println("Packet not for this receiver");
    }
  }
}
