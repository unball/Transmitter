/**********FIRMWARE PARA ESTUDO EM CONTROLE***********/
/* http://www-personal.umich.edu/~johannb/Papers/paper43.pdf
 *  Usando a ideia de cross coupling control
  */

  


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

                           
RF24 radio(CE,CS);                        // Set up nRF24L01 radio on SPI bus plus pins 3 & 2 for nano


const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };   // Radio pipe addresses for the 2 nodes to communicate.
uint64_t pipeEnvia=pipes[1];
uint64_t pipeRecebe=pipes[0];


ros::NodeHandle nh;

struct{
  int data1;
  int data2;
}myData;

struct{
  int motorA;
  int motorB;
}velocidades;

int menu;
void motorVel( const communication::comm_msg& velocidadesdata){
   menu = velocidadesdata.menu;
   velocidades.motorA = velocidadesdata.MotorA;
   velocidades.motorB = velocidadesdata.MotorB;
}

ros::Subscriber<communication::comm_msg> sub("radio_topic", &motorVel );

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
  switch (menu) {
      case 8:
      case 9:
        repeteVelocidade();
        break;
      
  }
}

void mandaVelocidadeZero(){
  velocidades.motorA=0;
  velocidades.motorB=0;
  radio.stopListening();
  radio.enableDynamicAck();
  radio.write(&velocidades,sizeof(velocidades), 1);
  radio.startListening();
}

void repeteVelocidade(){
  radio.stopListening();
  radio.enableDynamicAck();
  radio.write(&velocidades,sizeof(velocidades), 1);
  radio.startListening();
}

void mandaVelocidades(){
  Serial.print("Velocidade motor A:");
  while(!Serial.available());
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    delay(30);
  }
  velocidades.motorA=inString.toInt();
  Serial.println(velocidades.motorA);
  inString = "";
  Serial.print("Velocidade motor B:");
  while(!Serial.available());
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
      inString += (char)inChar;
    }
    delay(30);
  }
  velocidades.motorB=inString.toInt();
  inString = "";
  Serial.println(velocidades.motorB);

  radio.stopListening();
  radio.enableDynamicAck();
  radio.write(&velocidades,sizeof(velocidades), 1);
  radio.startListening();
}
void recebeMensagem(){
   if(radio.available()){
     while(radio.available()){       
      radio.read(&myData,sizeof(myData));
     }
     Serial.print("mensagem: ");
     Serial.println(myData.data1);
   }
}

void mandaMensagem(){
  Serial.println("Escreva 1 caractere");
  while(!Serial.available());
  
  char c = toupper(Serial.read());
  myData.data1=c;
  radio.stopListening();
  radio.enableDynamicAck();                 //essa funcao precisa andar colada na de baixo
  radio.write(&myData,sizeof(myData), 1);   //lembrar que precisa enableDynamicAck antes
                                            // 1-NOACK, 0-ACK
  radio.startListening(); 
  Serial.println(c);
}

void radio_status(){
  Serial.print("CE: ");
  Serial.println(CE);
  Serial.print("CS: ");
  Serial.println(CS);
  
  Serial.print("Channel: ");
  Serial.print(radio.getChannel());
  Serial.println("  0-125");
  Serial.print("PALevel: ");
  Serial.print(radio.getPALevel());
  Serial.println("  0-3");
  Serial.print("DataRate: ");
  Serial.print(radio.getDataRate());
  Serial.println("  0->1MBPS, 1->2MBPS, 2->250KBPS");
  Serial.println();
  Serial.print("Enviar por: ");
  Serial.println(int(pipeEnvia));
  Serial.print("Recebe por: ");
  Serial.println(int(pipeRecebe));
  Serial.print("Payload size: ");
  Serial.println(radio.getPayloadSize());
   

}

void plataforma(){
  Serial.println("Plataforma arduino nano");
  unsigned long time=millis();
  Serial.println(time);
}

void radioSetup(){
  radio.begin();                           // inicializa radio
  radio.setChannel(108);                   //muda para um canal de frequencia diferente de 2.4Ghz
  radio.setPALevel(RF24_PA_MAX);           //usa potencia maxima
  radio.setDataRate(RF24_2MBPS);           //usa velocidade maxima

  radio.openWritingPipe(pipeEnvia);        //escreve pelo pipe0
  radio.openReadingPipe(1,pipeRecebe);      //escuta pelo pipe1

  radio.enableDynamicPayloads();           //ativa payloads dinamicos(pacote tamamhos diferentes do padrao)
  radio.setPayloadSize(sizeof(velocidades));   //ajusta os o tamanho dos pacotes ao tamanho da mensagem
  
  radio.startListening();                 // Start listening
}


