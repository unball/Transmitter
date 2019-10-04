#include <SPI.h> //for radio
#include "RF24.h" // for radio
#include <ros.h>
#include <ArduinoHardware.h>
#include <main_system/robot_msg.h>

void radioSetup();
void repeteVelocidade();
bool readAndPrintRadio();


typedef struct {
    float vel_A;
    float vel_B;
    int32_t in_A, in_B;
    uint32_t time;
} vel;

#define NUMBER_OF_ROBOTS 3
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
  int16_t motorA;
  int16_t motorB;
  double Kp[2], Ki[2], Kd[2];
};

Velocidade velocidades[NUMBER_OF_ROBOTS];

int menu;
void motorVel( const main_system::robot_msg& velocidadesdata){
   menu = velocidadesdata.menu;
   for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    velocidades[i].motorA = velocidadesdata.MotorA[i];
    velocidades[i].motorB = velocidadesdata.MotorB[i];
    velocidades[i].Kp[0] = velocidadesdata.Kp[i];
    velocidades[i].Kp[1] = velocidadesdata.Kp[i+3];
    velocidades[i].Ki[0] = velocidadesdata.Ki[i];
    velocidades[i].Ki[1] = velocidadesdata.Ki[i+3];
    velocidades[i].Kd[0] = velocidadesdata.Kd[i];
    velocidades[i].Kd[1] = velocidadesdata.Kd[i+3];
   }
}

//ros::Subscriber<main_system::robot_msg> sub("radio_topic", &motorVel);

String inString = ""; 
/***************************************************************/

void setup(void) {
  Serial.begin(115200);
  radioSetup();
  /*nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);*/
}


void loop(){
  /*nh.spinOnce();
  for(uint16_t i=0 ; i<6000 ; i++){
    repeteVelocidade();
  }
  uint32_t t = millis();
  while(millis()-t < 3000){
    printData();
  }*/
  readAndPrintRadio();
}

void repeteVelocidade(){
  radio.stopListening();
  radio.enableDynamicAck();
    Serial.println("Mandando coisa");
  for (int i=0; i<NUMBER_OF_ROBOTS; i++) {
    radio.openWritingPipe(addresses[i]);
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

bool readAndPrintRadio(){ // recebe mensagem via radio, se receber uma mensagem retorna true, se não retorna false
    vel data;
    
    radio.startListening();

     if(radio.available()){
      while(radio.available()){
        data.time = 12345;
        data.vel_A = 12345;
        data.vel_B = 12345;
        radio.read(&data,sizeof(vel));
        Serial.print(data.time);
        Serial.print(",");
        Serial.print(data.in_A);
        Serial.print(",");
        Serial.print(data.vel_A);
        Serial.print(",");
        Serial.print(data.in_B);
        Serial.print(",");
        Serial.print(data.vel_B);
        Serial.print("\n");
      }
      return true;
     }
    return false;
  }