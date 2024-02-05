//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>


// Libraries for BMP280 and GPS
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define GPS pins
#define ARDUINO_GPS_RX 38
#define ARDUINO_GPS_TX 36
#define GPS_BAUD 9600

SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX);
TinyGPSPlus tinyGPS;

// define bmp280
#define BMP_SDA 21
#define BMP_SCL 22

Adafruit_BMP280 bmp;

float pressaoPadrao = 1013.25;
float offsetPressao = 0.0;
float offsetAltitude = 0.0;
int meuEndereco = 42;

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 915E6

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//packet counter
int counter = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);

  ssGPS.begin(GPS_BAUD);

  if (!bmp.begin(0x76)) {
    Serial.println("Failed to find BMP280");
  }


  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER ");
  display.display();
  
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.print("LoRa Initializing OK!");
  display.display();
  delay(2000);
}

void loop() {
   
  float pressao = bmp.readPressure() / 100.0 - offsetPressao;
  float altitude = bmp.readAltitude(pressaoPadrao) - offsetAltitude;
  float temperatura = bmp.readTemperature();

  Serial.print("Pressão: ");
  Serial.print(pressao);
  Serial.print(" Altitude: ");
  Serial.print(altitude);
  Serial.print(" Temperatura: ");
  Serial.println(temperatura);

  
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.write(meuEndereco);
  LoRa.print(altitude);
  LoRa.print(" ");
  LoRa.print(pressao);
  LoRa.print(" ");
  LoRa.print(temperatura);
  LoRa.print(" ");
  LoRa.print(counter);
  LoRa.endPacket();

  char stemp[120];
  sprintf(stemp, "%d, %lf, %lf, %lf, %d", meuEndereco, altitude, pressao, temperatura, counter);

  Serial.print(stemp);
  LoRa.print(stemp);


  display.clearDisplay();
  display.setCursor(0,0);
  display.println("LORA SENDER");
  display.setCursor(0,20);
  display.setTextSize(1);
  display.print("LoRa packet sent.");
  display.setCursor(0,30);
  display.print("Counter:");
  display.setCursor(50,30);
  display.print(counter);      
  display.display();

  counter++;
  
  delay(10000);
}
