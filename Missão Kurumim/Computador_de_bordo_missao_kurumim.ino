#include "DHT.h"

#define DHTPIN 13  
#define DHTTYPE DHT22   // DHT 22  
DHT dht(DHTPIN, DHTTYPE);

#include <TinyGPS++.h>
TinyGPSPlus tinyGPS; 

#define GPS_BAUD 9600  // GPS baud rate 

#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 36// GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 38 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Cria a SoftwareSerial

#define gpsPort ssGPS //GPS

#define SerialMonitor Serial

#include <Adafruit_BMP280.h>

//BMP 280 
#define BMP_SDA 21
#define BMP_SCL 22

Adafruit_BMP280 bmp;

//GY-521

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

// Endereço único do rádio

int meuEndereco = 42;  

//Char para envio de dados dos sensores da placa de computador de bordo

char stemp[400]; 

String dados;


void setup() 
{
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);

  //OPT101
  pinMode(sensorPin, INPUT);

  //Inicializa o DHT22 (sensor de temperatura e umidade)
  dht.begin(); 

  //Inicializa o BMP280 (sensor de pressão)
  if (!bmp.begin (0x76)) 
  {
    Serial.println("Sensor BMP280 não encontrado!");
  }

  //Inicializa o GY-521 (giroscópio e acelerômetro)
  if (!mpu.begin(0x68)) 
  {
    Serial.println("Sensor GY-521 não encontrado!");
  }

  //Seta os pinos SPI 
  SPI.begin(LORA_SCK,LORA_MISO,LORA_MOSI,LORA_SS);

  // Inicializa o rádio LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(915E6)) 
  {
    Serial.println("Falha ao inicializar o LoRa!");
    while (1);
  }
  
  Serial.println("LoRa Inicializado!");
  delay(1000);
}

void loop() 
{
  receberDadosDosSlaves(0); //Recebe os dados do suprimento de energia, que tem o endereço 0
  
  // Valores do OP101
  int sensorValue = analogRead(sensorPin);

 //DHT22

  float umidade = dht.readHumidity();           // lê a umidade 
  float temperaturaDHT = dht.readTemperature();  // lê a temperatura

  //BMP280

  float pressao = bmp.readPressure() / 100; // lê a pressão
  float altitude = bmp.readAltitude(1013.25); // lê a altitude


  //GY-521

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  float accelX = a.acceleration.x; // Acel X
  float accelY = a.acceleration.y; // Acel y
  float accelZ = a.acceleration.z; // Acel z


  float gyroX = g.gyro.x; // Giro x
  float gyroY = g.gyro.y; // Giro y
  float gyroZ = g.gyro.z; // Giro z


  //GPS
  
  float latitude = tinyGPS.location.lat(); //latitude
  float longitude = tinyGPS.location.lng(); //longitude
  float altitudeGPS = tinyGPS.altitude.meters(); //altitude
  int satellites = tinyGPS.satellites.value(); //número de satélites visíveis

  sprintf(stemp, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d, %.6f, %.6f, %.2f, %d",
         temperaturaDHT, umidade, pressao, altitude,
         accelX, accelY, accelZ,
         gyroX, gyroY, gyroZ,
         sensorValue, latitude, longitude, altitudeGPS, satellites); //char com todos os dados do computador de bordo

  enviarSensores(); //envia os dados do computador de bordo via LoRa
  
  delay(1500);

  enviarDadosParaSlave("Dados para Suprimento de Energia", 0); //Envia mensagem para o suprimento de energia enviar seus dados (corrente, tensão e temperatura das baterias)

  delay(1500);
}

void printGPSInfo() 
{
  // Printa as informações do gps 
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

// Delay para o GPS sincronizar e receber informações.
static void smartDelay(unsigned long ms) 
{
  unsigned long start = millis();
  do {
    // Se o gps começar a mandar informações
    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read()); 
  } while (millis() - start < ms);
}

// printDate() formato da data dd/mm/aa.
void printDate() 
{
  SerialMonitor.print(tinyGPS.date.day());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.month());
  SerialMonitor.print("/");
  SerialMonitor.print(tinyGPS.date.year());
}

// printTime() formato da hora "hh:mm:ss"
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

String gatherGPSInfo() 
{
  // Junta as informações do gps
  String gpsInfo = "Latitude: " + String(tinyGPS.location.lat(), 6) +
                   ", Longitude: " + String(tinyGPS.location.lng(), 6) +
                   ", Altitude: " + String(tinyGPS.altitude.meters()) +
                   ", Course: " + String(tinyGPS.course.deg()) +
                   ", Speed: " + String(tinyGPS.speed.mps()) +
                   ", Satellites: " + String(tinyGPS.satellites.value());

  return gpsInfo;
}

bool receberDadosDosSlaves(int endereco) 
{
  if (Serial.available() > 0) 
  {   
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    // Verifica o endereço de cada dispositivo e processe os dados recebidos de cada subsistema de acordo com o código deles
    if (enderecoRecebido == endereco) 
    {
        String dados = dadosRecebidos.substring(2);
        enviarResposta(dados);
        comandoRecebido = true; //flag que define que um comando foi recebido
        tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 100ms
    }
  }
  return comandoRecebido;
}

void processarDados(String nomeSlave, String dados) 
{
  Serial.println("Dados recebidos do " + nomeSlave + ": " + dados);
  
  enviarResposta(dados);
}

void enviarDadosParaSlave(String dados, int endereco) 
{
  Serial.print(endereco);  
  Serial.print(":");
  Serial.println(dados); // ex: "0:dados"
}

void enviarResposta(String resposta) 
{ 
  // Envia os dados recebidos dos outros subsistemas para a estação base
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Envia o endereço único, para a leitura da estação base
  LoRa.print(resposta);
  LoRa.endPacket();
}

void enviarSensores() 
{
  // Envia os dados dos sensores da placa do computador de bordo para a estação base
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Envia o endereço único, para a leitura da estação base
  LoRa.print(stemp);
  LoRa.endPacket();
  delay(1000);
}
