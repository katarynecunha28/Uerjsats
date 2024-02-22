/*
 * This is the controller code for the reaction wheel satellite model published
 * at https://charleslabs.fr/en/project-Reaction+Wheel+Attitude+Control
 *
 * The electronics diagram and the mechanical build is described in this article.
 *
 * You can set the PID controller terms and various other parameters hereunder.
 *
 * Charles GRASSIN, 2021
 * MIT License
 */

#define SERIAL_DEBUG_ENABLE 1 /* 0 = disable, 1 = enable */
#define MPU6050_CALIBRATION 0 /* 0 = disable, 1 = enable */
#define CONTROLLER_MODE 0 /* 0 = Speed stabilization only, 1 = Speed and Attitude stabilization, 2 = same as 1, but change set point every N secondes */
// ---COMUNICATION---

unsigned long tempoInicial = 0;
bool comandoRecebido = false;

// -------PINS-------
#define PIN_ENABLE 1
#define PIN_DIR    2
#define PIN_STEP   5
#define PIN_SLEEP  4
#define PIN_LED    13
#define ms0        6
#define ms1        7
#define ms2        8
// ------------------

// -----STEPPER------
#include "AccelStepper.h" // This is a hacked version of this library to allow
                          // speed control instead of position control.
AccelStepper myStepper(1, PIN_STEP, PIN_DIR);
double motorSpeed = 0;
#define MICROSTEPPING 1 /* 1 or 2 or 4 or 8 or 16 or 32     *///2
#define ACCELERATION 200 /* Steps per s */ //1200
#define MAX_SPEED (5000 * MICROSTEPPING) //5000

// ----SERVO/LDR----
#include <Servo.h>
const int servoPin = 9; // Pino do servo
const int ldrPin = A1;  // Pino do LDR
Servo myServo;
int anguloDesejado = 0;
int maxLightAngle = 0;

// -------PID--------
#include "PID.h"
/*
const double P_TERM = 0.050 * MICROSTEPPING;
const double I_TERM = 0.000 * MICROSTEPPING;
const double D_TERM = 0.017 * MICROSTEPPING; */
const double P_TERM = 0.050 * MICROSTEPPING;
const double I_TERM = 0.000 * MICROSTEPPING;
const double D_TERM = 0.017 * MICROSTEPPING; 
PIDController pidSpeed(P_TERM, I_TERM, D_TERM);
PIDAngleController pidAttitude(2.5, 0, 400);
// ------------------

// ------MPU6050-----
#include <Wire.h>
#define MPU_ADDRESS 0x68   // I2C address of the MPU-6050
double GYRO_ERROR = 61.96; // rollingAvg error compensation
double yawAngle=0, yawAngularSpeed=0;
// ------------------

// ------GENERAL-----
long timeCur, timePrev, timeStart; 
const int numReadings= 5;
double readings[numReadings];
int readIndex = 0;
double total = 0, rollingAvg = 0;
double targetAttitude = 0;
// ------------------

void setup() {
  delay(5000);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);

  //Microstepping
  pinMode(ms0, OUTPUT);
  digitalWrite(ms0, LOW);
  pinMode(ms1, OUTPUT);
  digitalWrite(ms1, LOW);
  pinMode(ms2, OUTPUT);
  digitalWrite(ms2, LOW);

  if(MICROSTEPPING==2){
      pinMode(ms0, HIGH);
  } else if (MICROSTEPPING==4){
      pinMode(ms1, HIGH);
  } else if (MICROSTEPPING==8){
      pinMode(ms1, HIGH);
      pinMode(ms0, HIGH);
  } else if (MICROSTEPPING==16){
      pinMode(ms2, HIGH);
  } else if (MICROSTEPPING==32){
      pinMode(ms2, HIGH);
      pinMode(ms0, HIGH);
  }
  delay(5000);
  #if SERIAL_DEBUG_ENABLE == 1
    Serial.begin(115200);
    Serial.println("Attitude Speed");
  #endif
  
  // Gyro setup
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // wakes up the MPU-6050
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x10); // 1000dps full scale
  Wire.endTransmission(true);

  // Gyro cal (only if selected, otherwise use saved value)
  #if MPU6050_CALIBRATION == 1
    calibrateMPU();
  #endif

 
  // Initial stepper parameters
  myStepper.setEnablePin (PIN_SLEEP);
  myStepper.setAcceleration(ACCELERATION);
  setSpeedStepper(0);
  
  timeCur = millis();
  timeStart = timeCur;

  //Servo and LDR
  myServo.attach(servoPin);
  myServo.write(90);

  //comunication
  receberDadosDoMaster(2);  // Escuta mensagens destinadas ao Slave Uno
  Serial.println("apos delay");
  Serial.println(comandoRecebido);
  
  if (comandoRecebido) {
    if (millis() - tempoInicial >= 100) {
      enviarDadosParaMaster("Velocidade angular:", yawAngularSpeed, 3);  // Envia para o Master
      comandoRecebido = false; // Reinicia a flag de comando recebido
      Serial.println("if concluido");     
    }
  }
}

// FSM variables
byte controllerState = 0;
int counts = 0;

// Main loop
void loop() {
  // Stop control after 40s
  // if(millis() - timeStart > 40000){
  //   digitalWrite(PIN_LED,0);
  //   myStepper.disableOutputs ();
  // }
  
  // Pulse stepper
  myStepper.run();

  // Every 10ms, read MPU and call controllers
  if(millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // Measure Gyro value
    yawAngularSpeed = ((double)readMPU()-GYRO_ERROR) / 32.8;
    yawAngle += (yawAngularSpeed * (timeCur - timePrev) / 1000.0);
    // Put angle between -180 and 180
    while (yawAngle <= -180) yawAngle += 360; 
    while (yawAngle > 180)   yawAngle -= 360;

    // Low Pass Filter the angular speed (https://www.arduino.cc/en/Tutorial/BuiltInExamples/Smoothing)
    total = total - readings[readIndex]; 
    readings[readIndex] = yawAngularSpeed;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings)
      readIndex = 0;
    rollingAvg = total / numReadings; 

    // Compute controller output
    #if CONTROLLER_MODE == 0
      // Detumbling only
      motorSpeed += pidSpeed.compute(0,rollingAvg,timeCur - timePrev);
    #else if CONTROLLER_MODE == 1 || CONTROLLER_MODE == 2
      // Change set point
      #if CONTROLLER_MODE == 2
        counts ++;
        if(counts == 250) {
          counts = 0;
          if(targetAttitude == 0)
            targetAttitude = 180;
          else
            targetAttitude = 0;
        }
      #endif
      
      // FSM transition
      if(controllerState == 1 &&  fabs(rollingAvg) > 360 /* °/s */){
        controllerState = 0;digitalWrite(PIN_LED,0);
      }
      else if(controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */)
        controllerState = 1;
      
      //FSM action
      if(controllerState == 0){
        motorSpeed += pidSpeed.compute(0,rollingAvg,timeCur - timePrev);
        pidAttitude.compute(targetAttitude,yawAngle,timeCur - timePrev);
      }
      else
        motorSpeed += pidSpeed.compute(pidAttitude.compute(targetAttitude,yawAngle,timeCur - timePrev),rollingAvg,timeCur - timePrev);
    #endif

    // Constrain speed to valid interval (saturation)
    if(motorSpeed > MAX_SPEED) motorSpeed = MAX_SPEED;
    else if (motorSpeed < -MAX_SPEED) motorSpeed = -MAX_SPEED;
    
    setSpeedStepper(motorSpeed);

    // Report attitude and speed
    
    #if SERIAL_DEBUG_ENABLE == 1
      Serial.print(yawAngle);
      Serial.print(" ");
      Serial.println(rollingAvg);
    #endif
  }

 
}


// Set the current speed and direction of the motor
void setSpeedStepper(double targetSpeed){
  if(targetSpeed > 0)
    myStepper.moveTo(1000000);
  else 
    myStepper.moveTo(-1000000);

  myStepper.setMaxSpeed(fabs(targetSpeed));
}

// Read a yaw angular speed value
int16_t readMPU(){
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS,2,true);
  return Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Calibrate the gyro by doing CALIBRATION_MEASUREMENTS_COUNT measurements
#define CALIBRATION_MEASUREMENTS_COUNT 200
void calibrateMPU(){
  GYRO_ERROR = 0;
  for(int i=0;i<CALIBRATION_MEASUREMENTS_COUNT;i++){
    GYRO_ERROR += readMPU();
    delay(20);
  }
  GYRO_ERROR = GYRO_ERROR/(double)CALIBRATION_MEASUREMENTS_COUNT;
  #if SERIAL_DEBUG_ENABLE == 1
    Serial.println(GYRO_ERROR);
  #endif
}

void receberDadosDoMaster(int endereco) {
  if (Serial.available() > 0) {
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco) {
      String dados = dadosRecebidos.substring(2);
      Serial.println("Dados recebidos do Master: " + dados);
      Serial.println("Dados recebidos do Master: " + dados);
      comandoRecebido = true; // Define que um comando foi recebido
      tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 10 segundos
    }
  }
}

void enviarDadosParaMaster(String dados, double valor, int endereco) {
  char buffer[20]; // buffer para armazenar a representação do double como uma sequência de caracteres
  dtostrf(valor, 6, 2, buffer); // converter o double para uma sequência de caracteres formatada com 6 dígitos no total, incluindo 2 dígitos após o ponto decimal
  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(": ");
  Serial.println(dados +" "+ valor);

}

int encontrarAnguloMaxLuz() {
  int maxLightValue = 0;
  int maxLightAngle = 0;
  myServo.write(0);
  delay(2000);

  // Girar o servo motor em 360 graus
  for (int angle = 0; angle <= 180; angle++) {
    myServo.write(angle); // Configurar o servo para o ângulo atual
    delay(15); // Aguardar o servo se mover

    int lightValue = analogRead(ldrPin); // Ler o valor do LDR

    // Verificar se o valor atual é maior que o máximo registrado
    if (lightValue > maxLightValue) {
      maxLightValue = lightValue;
      maxLightAngle = angle;
    }
  }

  return maxLightAngle;
}
int ajuste(int anguloDesejado) {
  if (anguloDesejado>90 && anguloDesejado<180){
      myServo.write(0);
      //girar para um sentido e nao precisa recarregar o angulo
      return (anguloDesejado);

  } else if (anguloDesejado>180 && anguloDesejado<270){
      myServo.write(180);
      //girar para outro sentido e recalcular o angulo
      return (anguloDesejado+180);

  } else if (anguloDesejado<90){
      //nao precisa girar mas tem que recalcular o angulo
      return (anguloDesejado-90);
      
  }
}
