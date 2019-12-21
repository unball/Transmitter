#include <SPI.h>
#include "RF24.h"
#include <ArduinoHardware.h>

/* Definições */
#define NUMBER_OF_ROBOTS 5

/* Declaração das funções */
void radioSetup();
void enviaMensagem();
void atualizaVelocidades();

/* Pinos para o rádio */
int CE = 3;
int CS = 2;

/* Objeto que gerencia o rádio */
RF24 radio(CE,CS);

/* Endereços */
uint64_t txPipeAddress = 0xABCDABCD71L;
uint64_t rxPipeAddress = 0x744d52687CL;

/* Estrutura para a mensagem a ser transmitida */
struct Velocidade{
  int16_t motorA[5];
  int16_t motorB[5];
};

/* Estrutura para a mensagem a ser recebida do USB */
struct VelocidadeSerial {
  Velocidade data;
  uint16_t checksum;
};

/* Mensagem a ser transmitida */
Velocidade velocidades;

/* Contagem de erros de transmissão via USB detectados */
uint32_t erros = 0;

/* Loop de setup */
void setup(void) {
  Serial.begin(115200);
  radioSetup();
}

/* Loop que é executado continuamente */
void loop(){
  atualizaVelocidades();
  enviaMensagem();
}

/* Envia a mensagem pelo rádio */
void enviaMensagem(){
  // Garante que o rádio não está escutando
  radio.stopListening();

  // Permite chamadas de write sem ACK
  radio.enableDynamicAck();

  // Abre um pipe para escrita
  radio.openWritingPipe(txPipeAddress);

  // Envia a mensagem
  radio.write(&velocidades,sizeof(velocidades), 1);
}

/* Configura o rádio */
void radioSetup(){
  // inicializa radio
  radio.begin();

  // muda para um canal de frequencia diferente de 2.4Ghz
  radio.setChannel(108);

  // usa potencia maxima
  radio.setPALevel(RF24_PA_MAX);

  // usa velocidade maxima
  radio.setDataRate(RF24_2MBPS);

  // escuta pelo pipe1
  radio.openReadingPipe(1,rxPipeAddress);

  // ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
  radio.enableDynamicPayloads();

  // ajusta o tamanho dos pacotes ao tamanho da mensagem
  radio.setPayloadSize(sizeof(velocidades));
  
  // Start listening
  radio.startListening();
}

/* Lê do serial novas velocidades */
void atualizaVelocidades(){
  int initCounter = 0;
  while(Serial.available()){
    /* Lê um caracter da mensagem */
    char character = Serial.read();

    /* Incrementa o contador se for 'B' */
    if(character == 'B') initCounter++;

    /* Se os três primeiros caracteres são 'B' então de fato é o início da mensagem */
    if(initCounter >= 3){
      VelocidadeSerial receber;
      
      /* Lê a mensagem até o caracter de terminação e a decodifica */
      Serial.readBytes((char*)(&receber), (size_t)sizeof(VelocidadeSerial));

      /* Faz o checksum */
      uint16_t checksum = 0;
      for(int i=0 ; i<5 ; i++){
        checksum += receber.data.motorA[i] + receber.data.motorB[i];
      }

      /* Verifica o checksum */
      if(checksum == receber.checksum){
        /* Copia para o buffer global de velocidades */
        velocidades = receber.data;

        /* Reporta que deu certo */
        Serial.println("OK");
      }
      else {
        /* Devolve o checksum calculado se deu errado */
        Serial.println(checksum);
      }

      /* Zera o contador */
      initCounter = 0;
    }
  }
}
