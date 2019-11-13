#include <SPI.h> //for radio
#include "RF24.h" // for radio
#include <ros.h>
#include <ArduinoHardware.h>
#include <ros_lib/ros_lib/main_system/robot_msg.h>

void radioSetup();
void repeteVelocidade();

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

/*const uint8_t addresses [6][5] = {{0xAB, 0xCD, 0xAB, 0xCD, 0x71},
                                                  {0x54, 0x4D, 0x52, 0x68, 0x72},
                                                  {0x54, 0x4D, 0x52, 0x68, 0x73},
                                                  {0x54, 0x4D, 0x52, 0x68, 0x74},
                                                  {0x54, 0x4D, 0x52, 0x68, 0x75},
                                                  {0x54, 0x4D, 0x52, 0x68, 0x76}};*/
const uint64_t addresses[4] = { 0xABCDABCD71L, 0x544d52687CL, 0x644d52687CL, 0x744d52687CL };
uint64_t pipeRecebe = addresses[3];

ros::NodeHandle nh;

struct Velocidade{
  int16_t motorA[5];
  int16_t motorB[5];
};

Velocidade velocidades;

int menu;
void motorVel( const main_system::robot_msg& velocidadesdata){
   menu = velocidadesdata.menu;
   for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    velocidades.motorA[i] = velocidadesdata.MotorA[i];
    velocidades.motorB[i] = velocidadesdata.MotorB[i];
   }
}

ros::Subscriber<main_system::robot_msg> sub("radio_topic", &motorVel);

String inString = ""; 
/***************************************************************/

void setup(void) {
  Serial.begin(115200);
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
  radio.openWritingPipe(addresses[0]);
  radio.write(&velocidades,sizeof(velocidades), 1);
  //velocidades[i].motorA = 0;
  //velocidades[i].motorB = 0;
}

void radioSetup(){
  radio.begin();                           // inicializa radio
  radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
  radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
  radio.setDataRate(RF24_2MBPS);           //usa velocidade maxima

  radio.openReadingPipe(1,pipeRecebe);      //escuta pelo pipe1

  radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
  radio.setPayloadSize(sizeof(velocidades));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
  
  radio.startListening();                 // Start listening
}