

unsigned long tempoInicial = 0;
bool comandoRecebido = false;



void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  receberDadosDoMaster(1);  // Escuta mensagens destinadas ao Slave Uno
  delay(1000);
  Serial.println("apos delay");
  Serial.println(comandoRecebido);
  
  if (comandoRecebido) {
    if (millis() - tempoInicial >= 100) {
      enviarDadosParaMaster("FOIIII", 3);  // Envia para o Master
      comandoRecebido = false; // Reinicia a flag de comando recebido
      Serial.println("if concluido");     
    }
  }
}

void receberDadosDoMaster(int endereco) {
  if (Serial1.available() > 0) {
    String dadosRecebidos = Serial1.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco) {
      String dados = dadosRecebidos.substring(2);
      Serial1.println("Dados recebidos do Master: " + dados);
      Serial.println("Dados recebidos do Master: " + dados);
      comandoRecebido = true; // Define que um comando foi recebido
      tempoInicial = millis(); // Salva o tempo inicial para enviar a resposta após 10 segundos
    }
  }
}

void enviarDadosParaMaster(String dados, int endereco) {
  Serial.print(endereco);  // Printar no monitor serial
  Serial.print(": ");
  Serial.println(dados);

  Serial1.print(endereco);  // Printar no monitor serial
  Serial1.print(": ");
  Serial1.println(dados);
}
