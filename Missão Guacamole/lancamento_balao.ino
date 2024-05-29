#include "DHT.h"

#define DHTPIN 14  
#define DHTTYPE DHT22   // DHT 22  
DHT dht(DHTPIN, DHTTYPE);

#include <TinyGPS++.h> // Include the TinyGPS++ library
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

#define GPS_BAUD 9600  // GPS module baud rate. 

#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 38// GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 36 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

// Define the serial monitor port.
#define SerialMonitor Serial

#include <Adafruit_BMP280.h>

//BMP 280 pins
#define BMP_SDA 21
#define BMP_SCL 22

Adafruit_BMP280 bmp;

//GY

#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

//LoRa 

#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Receiver Address

int myaddress = 42;

void setup() {
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);

  //Initialize DHT22
  dht.begin();  // Initialize DHT22 sensor

  //Initialize BMP280 sensor 
  if (!bmp.begin (0x76)) {
    Serial.println("Failed to find BMP280");
  }

  //Initialize MPU6050
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
  }

  //SPI LoRa pins
  SPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_SS);

  // Initialize LoRa module
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa initialization failed");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // OPT101 Values
  int sensorPin = 25;
  float sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);

  float umidade = dht.readHumidity();           // Read humidity from DHT22
  float temperaturaDHT = dht.readTemperature();  // Read temperature from DHT22

  Serial.print("Temperatura: " + String(temperaturaDHT) + " Umidade: " + String(umidade));


  float pressao = bmp.readPressure() / 100; // Read pressure from BMP280
  float altitude = bmp.readAltitude(1013.25); // Read altitude from BMP280

  Serial.println(" Pressão: " + String(pressao) + " Altitude: " + String(altitude));

  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  float accelX = a.acceleration.x; // Accel X
  float accelY = a.acceleration.y; // Accel y
  float accelZ = a.acceleration.z; // Accel z
  Serial.print("aceleracao: ");
  Serial.println(" x: " + String(accelX) + " y: " + String(accelY) + " z: " + String(accelZ));

  float gyroX = g.gyro.x; // Gyro x
  float gyroY = g.gyro.y; // Gyro y
  float gyroZ = g.gyro.z; // Gyro z
  Serial.print("giroscopio: ");
  Serial.println(" x: " + String(gyroX) + " y: " + String(gyroY) + " z: " + String(gyroZ));

  Serial.println();

  printGPSInfo();

  // Prepare data to be sent over LoRa
  String dataToSend = "Temperature: " + String(temperaturaDHT) + ", Humidity: " + String(umidade) +
                      ", Pressure: " + String(pressao) + ", Altitude: " + String(altitude) +
                      ", Acceleration(XYZ): " + String(accelX) + "," + String(accelY) + "," + String(accelZ) +
                      ", Gyroscope(XYZ): " + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) +
                      ", Sensor Value: " + String(sensorValue);

  // Convert String to char array
  char charBuf[dataToSend.length() + 1];
  dataToSend.toCharArray(charBuf, dataToSend.length() + 1);

  // Send data over LoRa
  LoRa.beginPacket();
  LoRa.write(myaddress);
  LoRa.print(charBuf);
  LoRa.endPacket();

  delay(1500);
}

void printGPSInfo() {
  // Print latitude, longitude, altitude in feet, course, speed, date, time,
  // and the number of visible satellites.
  SerialMonitor.println("GPS: ");
  SerialMonitor.print("Latitude: "); SerialMonitor.print(tinyGPS.location.lat(), 6);
  SerialMonitor.print(" Longitude: "); SerialMonitor.print(tinyGPS.location.lng(), 6);
  SerialMonitor.print(" Altitude: "); SerialMonitor.print(tinyGPS.altitude.meters());
  SerialMonitor.print(" Course: "); SerialMonitor.print(tinyGPS.course.deg());
  SerialMonitor.print(" Speed: "); SerialMonitor.print(tinyGPS.speed.mps());
  SerialMonitor.print(" Date: "); printDate();
  SerialMonitor.print(" Time: "); printTime();
  SerialMonitor.print(" Sats: "); SerialMonitor.println(tinyGPS.satellites.value());
  SerialMonitor.println("==================================");
}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

// printDate() formats the date into dd/mm/yy.
void printDate() {
  SerialMonitor.print(tinyGPS.date.day());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.month());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime() {
  SerialMonitor.print(tinyGPS.time.hour());
  SerialMonitor.print(":");
  if (tinyGPS.time.minute() < 10) SerialMonitor.print('0');
  SerialMonitor.print(tinyGPS.time.minute());
  SerialMonitor.print(":");
  if (tinyGPS.time.second() < 10) SerialMonitor.print('0');
  SerialMonitor.print(tinyGPS.time.second());
}

String gatherGPSInfo() {
  // Gather GPS information
  String gpsInfo = "Latitude: " + String(tinyGPS.location.lat(), 6) +
                   ", Longitude: " + String(tinyGPS.location.lng(), 6) +
                   ", Altitude: " + String(tinyGPS.altitude.meters()) +
                   ", Course: " + String(tinyGPS.course.deg()) +
                   ", Speed: " + String(tinyGPS.speed.mps()) +
                   ", Satellites: " + String(tinyGPS.satellites.value());

  return gpsInfo;
}
