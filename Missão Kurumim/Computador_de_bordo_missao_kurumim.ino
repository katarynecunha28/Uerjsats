#include "DHT.h"

#define DHTPIN 13  
#define DHTTYPE DHT22   // DHT 22  
DHT dht(DHTPIN, DHTTYPE);

#include <TinyGPS++.h> // Include the TinyGPS++ library
TinyGPSPlus tinyGPS; // Create a TinyGPSPlus object

#define GPS_BAUD 9600  // GPS module baud rate. 

#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 36// GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 38 // GPS RX, Arduino TX pin
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

//OPT101

const int sensorPin = 25;

//Tempo 
unsigned long tempoInicial;
bool comandoRecebido = false;

// Receiver Address

int meuEndereco = 42;  // Endereço do Computador de Bordo

int enderecoBase = 41; //Endereço da Estação Base

//Char para envio de dados dos sensores da placa de computador de bordo

char stemp[200]; 

String dados;

void setup() {
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);

  //Initialize DHT22
  dht.begin();  // Initialize DHT22 sensor

  //Initialize BMP280 sensor 
  if (!bmp.begin (0x76)) {
    Serial.println("Failed to find BMP280");
  }
  Serial.print("OI, EU SOU O BATMAN");
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

  enviarDadosParaSlave("Dados para Suprimento de Energia", 2);

  receberDadosDosSlaves(4);

  // OPT101 Values
  float sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);

 //DHT22

  float umidade = dht.readHumidity();           // Read humidity from DHT22
  float temperaturaDHT = dht.readTemperature();  // Read temperature from DHT22

  Serial.print("Temperatura: " + String(temperaturaDHT) + " Umidade: " + String(umidade));

  //BMP280

  float pressao = bmp.readPressure() / 100; // Read pressure from BMP280
  float altitude = bmp.readAltitude(1013.25); // Read altitude from BMP280

  Serial.println(" Pressão: " + String(pressao) + " Altitude: " + String(altitude));

  //MPU6050

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

  //GPS
  
  printGPSInfo();

  sprintf(stemp, "Temperature: %.2f, Humidity: %.2f, Pressure: %.2f, Altitude: %.2f, "
                      "Acceleration(XYZ): %.2f,%.2f,%.2f, Gyroscope(XYZ): %.2f,%.2f,%.2f, "
                      "Sensor Value: %d",
          temperaturaDHT, umidade, pressao, altitude,
          accelX, accelY, accelZ,
          gyroX, gyroY, gyroZ,
          sensorValue);
  

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

bool receberDadosDosSlaves(int endereco) {
  if (Serial.available() > 0) {   
    String dadosRecebidos = Serial.readStringUntil('\n');

    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    // Verifica o endereço de cada dispositivo e processe os dados recebidos de cada subsistema de acordo com o código deles
    switch (enderecoRecebido == endereco) {
      case 4:
        dados = dadosRecebidos.substring(1);
        Serial.println("Dados Suprimento de Energia: " + dados);
        enviarResposta(dados);
        tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 100ms
        return true; //flag que define que um comando foi recebido
      default:
        return false;  // Retorna false se o endereço não for reconhecido
        Serial.println("Endereço não reconhecido");
    }
  }
  return false;  // Retorna false se não receber os dados
  Serial.println("nao recebi");
}

void processarDados(String nomeSlave, String dados) {
  Serial.println("Dados recebidos do " + nomeSlave + ": " + dados);
  
  enviarResposta(dados);
}

void enviarDadosParaSlave(String dados, int endereco) {

  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(":");
  Serial.println(dados);
}

void enviarResposta(String resposta) { 
  // Envia os dados recebidos dos outros subsistemas para a estação base
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Envia o endereço do computador de bordo, para a leitura da estação base
  LoRa.print(resposta);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + resposta);
}

void enviarSensores() {
  // Envia os dados dos sensores da placa do computador de bordo para a estação base
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Envia o endereço do computador de bordo, para a leitura da estação base
  LoRa.print(stemp);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + String(stemp));
}
