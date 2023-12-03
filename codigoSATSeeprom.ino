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

class EE24CXXX {
  private:
    byte _device_address;
  public:
    EE24CXXX(byte device_address){ _device_address = device_address; }
    void write(unsigned int eeaddress, unsigned char * data, unsigned int data_len);
    void read (unsigned int eeaddress, unsigned char * data, unsigned int data_len);
    
    template <class T> int write(unsigned int eeaddress, const T& value);
    template <class T> int read(unsigned int eeaddress, T& value);
};

void EE24CXXX::write(unsigned int eeaddress, unsigned char * data, unsigned int data_len) {
  unsigned int  address;
  unsigned int  page_space;
  unsigned int  page=0;
  unsigned int  num_writes;
  unsigned char first_write_size;
  unsigned char last_write_size;  
  unsigned char write_size;  
  
  page_space = int(((eeaddress/64) + 1)*64)-eeaddress;      // Calculate space available in first page

  if (page_space>16) {                                      // Calculate first write size
     first_write_size = page_space-((page_space/16)*16);
     if (first_write_size==0) { first_write_size=16; }
  } else {
    first_write_size = page_space; 
  }
  if (data_len>first_write_size)  { last_write_size = (data_len-first_write_size)%16;   }      // calculate size of last write  
  num_writes = (data_len>first_write_size) ? ((data_len-first_write_size)/16)+2 : 1;           // Calculate how many writes we need
     
  unsigned char i=0, counter=0;
  address = eeaddress;
  for (page=0; page<num_writes; page++)  {
    if (page == 0) { 
      write_size = first_write_size; 
    } else if(page == (num_writes-1)) { 
      write_size = last_write_size;
    } else { 
      write_size = 16;
    }
 
    Wire.beginTransmission(_device_address);
    Wire.write((int)((address) >> 8));   // MSB multiplica por 2^8 = 256
    Wire.write((int)((address) & 0xFF)); // LSB
    counter = 0;
    do { 
      Wire.write((byte) data[i++]);
      counter++;
    } while(counter < write_size);
    Wire.endTransmission();
    address += write_size;   // Increment address for next write
     
    delay(5);  // needs 5ms for page write
  }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char * data, unsigned int data_len){
  unsigned char i = 0;
  unsigned int size = data_len;
  unsigned int j=0;
  while (size > 0){
    Wire.beginTransmission(_device_address);
    eeaddress += j*28;
    
    Wire.write((int)(eeaddress >> 8));   // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    
    if (size >= 28) { 
      Wire.requestFrom(_device_address, (unsigned int) 28);
      size -= 28;
    } else {
      Wire.requestFrom(_device_address, (unsigned int) size);
      size = 0;
    }
    while(Wire.available()) { data[i++] = Wire.read(); }
    j++;
  }
}

template <class T> int EE24CXXX::write(unsigned int eeaddress, const T& value) {
  const byte* p = (const byte*)(const void*)&value;
  unsigned char data[sizeof(value)+1];
  for (int i=0; i<sizeof(value); i++) { data[i] = *p++; }
  data[sizeof(value)] = '\n';
  write(eeaddress, data, sizeof(value));
  return sizeof(value);
}

template <class T> int EE24CXXX::read(unsigned int eeaddress, T& value) {
  byte * p = (byte*)(void*)&value;
  unsigned char c[sizeof(value)];
  read(eeaddress, c, sizeof(value));
  for (int i=0; i<sizeof(value); i++) { *p++ = c[i]; }
  return sizeof(value);
}
/*************************************************************************************************************
*******************************FIM - CLASSE EE24CXXX - CI I2C EEPROM AT24C128/256*****************************
**************************************************************************************************************/
EE24CXXX m(0x50);
int i =0;
struct dadosVoo {
  float valorSensor;
  float umidade; float temperaturaDHT;
  float pressao; float altitude;
  float accelX; float accelY; float accelZ;
  float gyroX; float gyroY;float gyroZ;
};

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

void loop(){
  
  //Valores OPT101
  float sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  

  // Medidas DHT22
  float umidade = dht.readHumidity();           // Read humidity from DHT22
  float temperaturaDHT = dht.readTemperature();  // Read temperature from DHT22

  Serial.print( "Temperatura: " + String(temperaturaDHT) + " Umidade: " + String(umidade));

  // Medidas BMP280
  float pressao = bmp.readPressure()/100;
  float altitude = bmp.readAltitude(1013.25);

  Serial.println(" Pressão: " + String(pressao) + " Altitude: " + String(altitude));

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

  // criando struct dadosVoo para armazenar os valores do dados do voo.
  dadosVoo voo;

  voo.valorSensor = sensorValue;
  voo.umidade = umidade;
  voo.temperaturaDHT = temperaturaDHT;
  voo.pressao = pressao;
  voo.accelX = accelX;
  voo.accelY = accelY;
  voo.accelZ = accelZ;
  voo.gyroX = gyroX;
  voo.gyroY = gyroY;
  voo.gyroZ = gyroZ;

  // 44 bytes 
  m.write(44*i, voo);     
  i++;
  delay(1500);
  while(1 && i==727){}
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