void setup() {
  Serial.begin(9600);  // Configura a taxa de comunicação serial
}

void loop() {
  if (Serial.available() > 0) {
    // Se há dados disponíveis na porta serial
    String data = Serial.readStringUntil('\n');  // Lê uma linha de dados até o caractere de nova linha (\n)
    
    // Processa os dados
    int area, centroidX, centroidY;
    sscanf(data.c_str(), "%d,%d,%d", &area, &centroidX, &centroidY);

    // Faça o que for necessário com os dados lidos
    Serial.print("Área: ");
    Serial.println(area);
    Serial.print("Centroide X: ");
    Serial.println(centroidX);
    Serial.print("Centroide Y: ");
    Serial.println(centroidY);
  }
}
