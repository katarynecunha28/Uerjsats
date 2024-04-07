
unsigned long tempoInicial; // Variável para armazenar o tempo inicial

#include <LoRa.h> // Inclui a biblioteca LoRa
#include <Arduino_MKRENV.h> // Inclui a biblioteca Arduino MKR ENV
#include <MKRENV.h> // Inclui a biblioteca MKR EN



int meuEndereco = 42;  // Endereço do receptor
int pinRele = 7; // Pino ao qual o relé está conectado

char stemp[120]; // String para armazenar os dados dos sensores

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial com o computador
  Serial1.begin(9600); // Inicializa a comunicação serial com o Arduino MKR

  pinMode(pinRele, OUTPUT); // Configura o pino do relé como saída

  while(!Serial); // Aguarda a conexão serial
  
  if (!LoRa.begin(433E6)) { // Inicializa o módulo LoRa na frequência especificada
    Serial.println("Starting LoRa failed!"); // Mensagem de erro se a inicialização falhar
    while (1); // Loop infinito
  }
  Serial.println("LoRa Initializing OK!");

  if (!ENV.begin()) { // Inicializa o sensor MKR ENV
    Serial.println("Failed to initialize MKR ENV shield!"); // Mensagem de erro se a inicialização falhar
    while (1); // Loop infinito
  }
}

void loop() {
  int packetSize = LoRa.parsePacket(); // Verifica se há pacotes recebidos
  if (packetSize) {
    int senderAddress = LoRa.read(); // Lê o endereço do remetente
    if (senderAddress == meuEndereco) {
      String comando = LoRa.readString(); // Lê o comando enviado pelo remetente
      Serial.println("Comando recebido: " + comando); // Imprime o comando recebido
      processarComando(comando);
    }
  }
  receberDadosDosSlaves(); // Recebe dados dos outros subsistemas

  // Ler todos os valores dos sensores do MKR ENV
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();


  // Construir a string para enviar via LoRa

  sprintf(stemp, "%d, %f, %f, %f, %f", meuEndereco, temperature, humidity, pressure, illuminance);

}

void processarComando(String comando) {
  //Serial.println("foi");
  if (comando.equals("A")) {
    enviarDadosParaSlave("Ligar motor", 1); //comando para controle de atitude

    delay(100);
  } else if (comando.equals("B")) {
    enviarDadosParaSlave("Dados Suprimento de Energia", 2); //comando para suprimento de energia
    delay(100);
  } else if (comando.equals("C")) {
    enviarSensores(); // envia os dados do MKR ENV
    delay(100);
  }
    else if (comando.equals("D")){
      enviarComandoParaPython("Dados Portenta", "5"); //comando para a missão
    }
    else if (comando.equals("E")){
      digitalWrite(pinRele, HIGH);
      delay(100);
      digitalWrite(pinRele, LOW);
      Serial.println("Relé acionado.");
      enviarResposta("Relé acionado.");
      delay(100); //comando para acionar relé
    }
    else if (comando.equals("F")){
      enviarDadosParaSlave("Mudar angulo", 1); //comando para mudar ângulo
    }
    else {
    // Se outra coissa for digitada no serial monitor ele printa: Comando não reconhecido
    enviarResposta("Comando não reconhecido.");
    return;
  }

}



bool receberDadosDosSlaves() {
  if (Serial1.available() > 0 || Serial.available() > 0) {
    String dadosRecebidos;
    if (Serial1.available() > 0) {
      dadosRecebidos = Serial1.readStringUntil('\n');
    } else {
      dadosRecebidos = Serial.readStringUntil('\n');
    }

    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    // Verifica o endereço do dispositivo e processa os dados recebidos de acordo
    switch (enderecoRecebido) {
      case 3:
        processarDados("Controle de Atitude", dadosRecebidos.substring(1));
        return true;
      case 4:
        processarDados("Suprimento", dadosRecebidos.substring(1));
        return true;

      case 6:
        processarDados("Portenta", dadosRecebidos.substring(1));
        return true;

      default:
        return false;  // Retorna false se o endereço não for reconhecido
    }
  }
  return false;  // Retorna false se não receber os dados
  Serial.println("nao recebi");
}

void processarDados(String nomeSlave, String dados) {
  Serial1.println("Dados recebidos do " + nomeSlave + ": " + dados);
  Serial.println("Dados recebidos do " + nomeSlave + ": " + dados);
  
  enviarResposta(dados);
}

void enviarDadosParaSlave(String dados, int endereco) {
  Serial1.print(endereco);  // Identificador de destino
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(":");
  Serial.println(dados);
}

void enviarResposta(String resposta) {
  // Envia a resposta de volta para o transmissor
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Endereço do receptor
  LoRa.print(resposta);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + resposta);
}

void enviarSensores() {
  // Envia a resposta de volta para o transmissor
  LoRa.beginPacket();
  LoRa.write(meuEndereco); // Endereço do receptor
  LoRa.print(stemp);
  LoRa.endPacket();

  Serial.println("Resposta enviada: " + String(stemp));
}

void enviarComandoParaPython(String endereco, String dados){
  Serial1.print(endereco);  // Identificador de destino
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(":");
  Serial.println(dados);
}

void receberDadosDoPython() {
  if (Serial1.available() > 0) {
    String dadosRecebidos = Serial1.readStringUntil('\n');
   

    // Processar os dados recebidos do Python conforme necessário
    Serial1.print("Dados recebidos do Portenta:");
    Serial.print("Dados recebidos do Python: ");
    Serial1.println(dadosRecebidos);
    Serial.println(dadosRecebidos);
    enviarResposta(dadosRecebidos);

  }
}
