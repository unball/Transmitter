#include <SPI.h>
#include "RF24.h"

/* Definições */
#define NUMBER_OF_ROBOTS 2
#define CONTROL_ID false

/* Pinos para o rádio */
int CE = 12;
int CS = 13;

/* Objeto que gerencia o rádio */
RF24 radio(CE,CS);

/* Endereços */
uint64_t txAddresses[] = {0xABCDABCD71LL, 0x544d52687CLL, 0x644d52687CLL};
uint64_t txPipeAddress = 0xABCDABCD71L;
uint64_t rxPipeAddress = 0x744d52687CL;

/* Estrutura para a mensagem a ser transmitida para o robô via rádio */
struct Velocidade{
  int16_t v[5];
  int16_t w[5];
};

struct VelocidadeRadio{
  int16_t v;
  int16_t w;
};

/* Estrutura para a mensagem a ser recebida do robô via rádio*/
struct reportStruct{
    uint32_t time;
    float v,w;
    int16_t va,vb;
    float enca, encb;
    float imuw;
};

/* Estrutura para a mensagem a ser recebida do USB */
struct VelocidadeSerial {
  Velocidade data;
  int16_t checksum;
};

/* Declaração das funções */
void radioSetup();
void sendData();
bool receiveData(reportStruct *);
void receiveUSBdata();

/* Mensagem a ser transmitida */
Velocidade velocidades;

/* Contagem de erros de transmissão via USB detectados */
uint32_t erros = 0;
uint32_t lastOK = 0;

/* Loop de setup */
void setup(void) {
  Serial.begin(115200);
  while(!Serial);
  radioSetup();
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Loop que é executado continuamente */
void loop(){
  #if CONTROL_ID

    static struct reportStruct reportData;

    // Recebe mensagem do rádio
    if(receiveData(&reportData)){
      // Envia pela serial USB
      Serial.printf("%d %lf %lf %d %d %lf %lf %lf\n", reportData.time, reportData.v, reportData.w, reportData.va, reportData.vb, reportData.enca, reportData.encb, reportData.imuw);
    }

  #else

    // Recebe velocidades via USB
    receiveUSBdata();

    // Envia via rádio
		static int32_t t = micros();
		if(micros()-t >= 500){
			t = micros();
			sendData();
		}

    // Acende o LED se recebeu mensagem do USB em menos de 5ms
    if(millis()-lastOK < 5){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
    }
    
  #endif

}

/* Envia a mensagem pelo rádio */
void sendData(){
  // Garante que o rádio não está escutando
  radio.stopListening();

  // Permite chamadas de write sem ACK
  radio.enableDynamicAck();

  // Envia a mensagem
  bool result = true;

  for(uint8_t i=0 ; i<NUMBER_OF_ROBOTS ; i++){
    VelocidadeRadio vel = {.v = velocidades.v[i], .w = velocidades.w[i]};
    
    // Abre o pipe para escrita
    radio.openWritingPipe(txAddresses[i]);

    // Envia
    result &= radio.write(&vel,sizeof(VelocidadeRadio), 1);
  }
  if(result){
    lastOK = millis();
  }
}  

/* Recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false */
bool receiveData(reportStruct *ret){
  if(radio.available()){
    radio.read(ret, sizeof(reportStruct));
    return true;
  }
  return false;
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
  radio.setPayloadSize(sizeof(VelocidadeRadio));
  
  // Start listening
  radio.startListening();

  #if !CONTROL_ID
    radio.stopListening();
  #endif
}

/* Lê do serial novas velocidades */
void receiveUSBdata(){
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
      int16_t checksum = 0;
      for(int i=0 ; i<5 ; i++){
        checksum += receber.data.v[i] + receber.data.w[i];
      }

      /* Verifica o checksum */
      if(checksum == receber.checksum){
        /* Copia para o buffer global de velocidades */
        velocidades = receber.data;

        /* Reporta que deu certo */
        Serial.printf("%d\t%d\t%d\n", checksum, velocidades.v[0], velocidades.w[0]);
        
      }
      else {
        /* Devolve o checksum calculado se deu errado */
        for(uint16_t i=0 ; i<sizeof(VelocidadeSerial) ; i++){
          Serial.printf("%p ", ((char*)&receber)[i]);
        }
        Serial.println("");
        //Serial.printf("%p\t%p\t%p\n", checksum, velocidades.v[0], velocidades.w[0]);
      }

      /* Zera o contador */
      initCounter = 0;
    }
  }
}
