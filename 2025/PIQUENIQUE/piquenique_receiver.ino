#include "Arduino.h"
#include "LoRaWan_APP.h"
#include "HT_SSD1306Wire.h"

// Configurações do LoRa
#define RF_FREQUENCY           915000000 // Frequência em Hz (915 MHz)
#define LORA_BANDWIDTH         0         // [0: 125 kHz]
#define LORA_SPREADING_FACTOR  7         // [SF7..SF12]
#define LORA_CODINGRATE        1         // [1: 4/5]
#define LORA_PREAMBLE_LENGTH   8         // Preambulo para Tx e Rx
#define LORA_SYMBOL_TIMEOUT    0         // Timeout em símbolos
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON   false

// Tamanho do buffer
#define BUFFER_SIZE            30 // Tamanho do payload esperado

// Variáveis do LoRa
char rxpacket[BUFFER_SIZE];
int16_t Rssi, rxSize;

// Funções de evento do LoRa
static RadioEvents_t RadioEvents;
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

// Display OLED
SSD1306Wire factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void lora_init() {
    // Inicializa o LoRa
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    Rssi = 0;

    // Define os eventos do LoRa
    RadioEvents.RxDone = OnRxDone;

    // Inicializa o rádio LoRa
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

    // Configuração do LoRa para recepção
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    // Ativa o modo de recepção
    Radio.Rx(0);
    Serial.println("LoRa inicializado e aguardando pacotes...");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    // Copia os dados recebidos para o buffer e adiciona um terminador nulo
    rxSize = size;
    Rssi = rssi;
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';

    // Verifica se o primeiro byte do payload é o endereço esperado (42)
    if (payload[0] == 42) {  // Endereço correspondente ao transmissor
        // Exibe os dados recebidos no Serial Monitor
        Serial.println("\nPacote recebido do endereço 42:");
        Serial.printf("Mensagem: %s\n", (char*)&payload[1]); // Ignora o primeiro byte (endereço)
        Serial.printf("RSSI: %d dBm, Tamanho: %d bytes\n", Rssi, rxSize);

        // Exibe os dados no display OLED
        factory_display.clear();
        factory_display.drawString(0, 0, "Pacote Recebido:");
        factory_display.drawString(0, 10, "Endereco: 42");
        factory_display.drawString(0, 20, "Mensagem:");
        factory_display.drawString(0, 30, String((char*)&payload[1]));
        factory_display.display();
    } else {
        // Ignora pacotes de outros endereços
        Serial.printf("Mensagem ignorada (endereco %d desconhecido)\n", payload[0]);
    }

    // Volta para o modo de recepção
    Radio.Rx(0);
}



void setup() {
    // Inicializa a comunicação serial
    Serial.begin(115200);

    // Previne falhas no barramento I2C
    Wire.begin(SDA_OLED, SCL_OLED);

    // Inicializa o display OLED
    factory_display.init();
    factory_display.clear();
    factory_display.display();
    factory_display.drawString(0, 0, "Iniciando...");
    factory_display.display();

    // Inicializa o LoRa
    lora_init();

    // Mensagem inicial no display
    delay(1000);
    factory_display.clear();
    factory_display.drawString(0, 0, "Aguardando pacotes...");
    factory_display.display();
}

void loop() {
    // Processa interrupções do rádio
    Radio.IrqProcess();

    // Evita que o watchdog reinicie o dispositivo
    delay(10);
}
