#include "DHT.h"

#define DHTPIN 13  
#define DHTTYPE DHT22   // DHT 22  
DHT dht(DHTPIN, DHTTYPE);

#include <TinyGPS.h>
#include <SoftwareSerial.h>

//GPS 
#define GPS_RX 36
#define GPS_TX 38
#define GPS_Serial_Baud 9600

TinyGPS gps;

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

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
  
  //GPS
  gpsSerial.begin(GPS_Serial_Baud);

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


  // Atualiza as informações do GPS
  
    bool newData = false;
    unsigned long chars;
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      // Serial.write(c); //apague o comentario para mostrar os dados crus
      if (gps.encode(c)) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    float latitude = flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat; //float latitude
    float longitude = flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon; //float longitude
    int sats = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites(); //int número de satélites
    

     sprintf(stemp, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d, %.6f, %.6f, %d",
                temperaturaDHT, umidade, pressao, altitude,
                accelX, accelY, accelZ,
                gyroX, gyroY, gyroZ,
                sensorValue, latitude, longitude, sats); //char com todos os dados do computador de bordo com os dados do GPS
  }
  else
  {
     sprintf(stemp, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %d",
            temperaturaDHT, umidade, pressao, altitude,
            accelX, accelY, accelZ,
            gyroX, gyroY, gyroZ,
            sensorValue); //char com todos os dados do computador de bordo sem o GPS, caso ele falhe
  }

  enviarSensores(); //envia os dados do computador de bordo via LoRa
  
  delay(1500);

  enviarDadosParaSlave("Dados para Suprimento de Energia", 0); //Envia mensagem para o suprimento de energia enviar seus dados (corrente, tensão e temperatura das baterias)

  delay(1500);
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
