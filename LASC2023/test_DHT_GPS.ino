

#include "DHT.h"

#define DHTPIN 14  

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);

#include <TinyGPS++.h> // Include the TinyGPS++ library
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

#define GPS_BAUD 9600  // GPS module baud rate. GP3906 defaults to 9600.

#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 38// GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 36 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

// Define the serial monitor port. On the Uno, and Leonardo this is 'Serial'
//  on other boards this may be 'SerialUSB'
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

//OPT 101

#include <Wire.h>
#include <SPI.h>

int sensorPin = 25;


void setup() {
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);

  //Initialize DHT22
  dht.begin();  // Initialize DHT22 sensor

  //Initialize BMP280 sensor 
  if (!bmp.begin (0x76)) {
    Serial.println("Failed to find BMP280");}

  //Inicialização GY-521
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
  }
}

void loop()
{
  delay(10000);

  //Valores OPT101
  float sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  

  // Medidas DHT22
  float umidade = dht.readHumidity();           // Read humidity from DHT22
  float temperaturaDHT = dht.readTemperature();  // Read temperature from DHT22

  Serial.print( "Temperatura: " + String(temperaturaDHT) + " Umidade: " + String(umidade));

  // Medidas BMP280
  float pressure = bmp.readPressure()/100;
  float altitude = bmp.readAltitude(1013.25);

  Serial.println(" Pressão: " + String(pressure) + " Altitude: " + String(altitude));

  // Medidas GY 521
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // GY valores

  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  Serial.print("aceleracao: ");
  Serial.println(" x: " + String(accelX) + " y: " + String(accelY) + " z: " + String(accelZ));

  float gyroX = g.gyro.x;
  float gyroY = g.gyro.y;
  float gyroZ = g.gyro.z;
  Serial.print("giroscopio: ");
  Serial.println(" x: " + String(gyroX) + " y: " + String(gyroY) + " z: " + String(gyroZ));
  
  Serial.println();

  printGPSInfo();

  

  delay(1000);
}
 /*while(true) {

 }*/
void printGPSInfo()
{
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
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}

// printDate() formats the date into dd/mm/yy.
void printDate()
{
  SerialMonitor.print(tinyGPS.date.day());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.month());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.year());
}

// printTime() formats the time into "hh:mm:ss", and prints leading 0's
// where they're called for.
void printTime()
{
  SerialMonitor.print(tinyGPS.time.hour());
  SerialMonitor.print(":");
  if (tinyGPS.time.minute() < 10) SerialMonitor.print('0');
  SerialMonitor.print(tinyGPS.time.minute());
  SerialMonitor.print(":");
  if (tinyGPS.time.second() < 10) SerialMonitor.print('0');
  SerialMonitor.print(tinyGPS.time.second());
}