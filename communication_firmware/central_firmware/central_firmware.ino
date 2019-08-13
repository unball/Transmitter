#include <SPI.h> //for radio
#include "RF24.h" // for radio
#include <ros.h>
#include <ArduinoHardware.h>
#include <communication/comm_msg.h>

#define NUMBER_OF_ROBOTS 5
#define NANO
#ifndef NANO
#define PRO_MICRO
#endif
/*************  USER Configuration *****************************/
/***********radio***********/
/*Radio pins*/
#ifdef PRO_MICRO
int CE = 10;
int CS = A0;
#else //nano
int CE = 3;
int CS = 2;
#endif
                           
RF24 radio(CE,CS);

//const uint64_t pipes[4] = { 0xABCDABCD71LL, 0x544d52687CLL , 0x644d52687CLL, 0x744d52687CLL};
const uint8_t adresses [NUMBER_OF_ROBOTS+1][5] = {{0xAB, 0xCD, 0xAB, 0xCD, 0x71},
                                                  {0x54, 0x4D, 0x52, 0x68, 0x7C},
                                                  {0x64, 0x4D, 0x52, 0x68, 0x7C},
                                                  {0x74, 0x4D, 0x52, 0x68, 0x7C},
                                                  {0x84, 0x4D, 0x52, 0x68, 0x7C},
                                                  {0x94, 0x4D, 0x52, 0x68, 0x7C}};
uint8_t pipeEnvia[5] = adresses[0];
uint8_t pipeRecebe[5] = pipes[5];

ros::NodeHandle nh;

struct Velocidade{
  int16_t motorA;
  int16_t motorB;
};

Velocidade velocidades[NUMBER_OF_ROBOTS];

int menu;
void motorVel( const communication::comm_msg& velocidadesdata){
   menu = velocidadesdata.menu;
   for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    velocidades[i].motorA = velocidadesdata.MotorA[i];
    velocidades[i].motorB = velocidadesdata.MotorB[i];
   }
}

ros::Subscriber<communication::comm_msg> sub("radio_topic", &motorVel);

String inString = ""; 
/***************************************************************/

void setup(void) {
  radioSetup();
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}


void loop(){
  nh.spinOnce();
  repeteVelocidade();
}

void repeteVelocidade(){
  radio.stopListening();
  radio.enableDynamicAck();
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    radio.openWritingPipe(adresses[i]);
    radio.write(&velocidades[i],sizeof(velocidades[i]), 1);
    velocidades[i].motorA = 0;
    velocidades[i].motorB = 0;  
  }
}

void radioSetup(){
  radio.begin();                           // inicializa radio
  radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
  radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
  radio.setDataRate(RF24_2MBPS);           //usa velocidade maxima

  radio.openReadingPipe(1,pipeRecebe);      //escuta pelo pipe1

  radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
  radio.setPayloadSize(sizeof(velocidades[0]));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
  
  radio.startListening();                 // Start listening
}
