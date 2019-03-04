#include <SPI.h> //for radio
#include "RF24.h" // for radio
#include <ros.h>
#include <ArduinoHardware.h>
#include <communication/comm_msg.h>
/*************  USER Configuration *****************************/
/***********radio***********/
/*Radio pins*/
//int CE = 10; //pro micro
//int CS = A0; //pro micro
int CE = 3; //nano
int CS = 2; //nano
                           
RF24 radio(CE,CS);

const uint64_t pipes[4] = { 0xABCDABCD71LL, 0x544d52687CLL , 0x644d52687CLL, 0x744d52687CLL};
uint64_t pipeEnvia=pipes[0];
uint64_t pipeRecebe=pipes[3];

ros::NodeHandle nh;

struct Velocidade{
  int16_t motorA;
  int16_t motorB;
};

Velocidade velocidades[3];

int menu;
void motorVel( const communication::comm_msg& velocidadesdata){
   menu = velocidadesdata.menu;
   for (int i=0; i<3; i++) {
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
  for (int i=0; i<3; i++) {
    radio.openWritingPipe(pipes[i]);
    radio.write(&velocidades[i],sizeof(velocidades[i]), 1);
    velocidades[i].motorA = 0;
    velocidades[i].motorB = 0;  
  }
  radio.startListening();
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
