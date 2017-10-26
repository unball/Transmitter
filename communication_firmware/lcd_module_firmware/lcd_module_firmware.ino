#include <SPI.h>
#include "RF24.h"

struct DataStruct{
    int Robot=0;
    int Message=0;
};

DataStruct Report;

int CE = A0; //pro micro
int CS = 10; //pro micro

RF24 radio(CE,CS);

const uint64_t Channels[4] = { 0xABCDABCD71LL, 0x544d52687CLL , 0x644d52687CLL, 0x744d52687CLL};
uint64_t ChannelEnvia = Channels[1];
uint64_t ChannelRecebe = Channel[0];

void setup() {
  // put your setup code here, to run once:
  RadioSetup();

}

void loop() {
  // put your main code here, to run repeatedly:
  ReceiveReport(&Report);

}

void RadioSetup(){
    radio.begin();                           // inicializa radio
    radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
    radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
    radio.setDataRate(RF24_2MBPS);           //usa velocidade de transmissao maxima

    radio.openWritingPipe(ChannelEnvia);        //escreve pelo pipe0 SEMPRE
    radio.openReadingPipe(1,ChannelRecebe);      //escuta pelo pipe1, evitar usar pipe0

    radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
    radio.setPayloadSize(sizeof(DataStruct));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
                                                 //tamanho maximo de payload 32 BYTES
    
    radio.startListening();                 // Start listening
}

void ReceiveReport(DataStruct *data){       //Function to receive data
  if(radio.available()){
    radio.read(data, sizeof(DataStruct));
  }
}






