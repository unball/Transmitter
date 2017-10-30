#include <SPI.h>
#include "RF24.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

struct DataStruct{
    int Robot;
    int Message;
};

DataStruct Report;

int CE = A0; //pro micro
int CS = 10; //pro micro

RF24 radio(CE,CS);
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


const uint64_t Channels[4] = { 0xABCDABCD71LL, 0x544d52687CLL , 0x644d52687CLL, 0x744d52687CLL};
uint64_t ChannelEnvia = Channels[0];
uint64_t ChannelRecebe = Channels[3];

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  RadioSetup();
  Report.Robot = 1;
  Report.Message = 1;
  lcd.begin(16,2);
  lcd.setBacklight (HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  //ReceiveReport();
  Serial.println(Report.Robot);
  if (Report.Robot == 0){
    lcd.setCursor (5,0);
    lcd.print ("UNBALL");
  }
  if (Report.Robot == 1){
    lcd.setCursor (5,0);
    lcd.print ("ROBOT 0");
  }
  if (Report.Robot == 2){
    lcd.setCursor (5,0);
    lcd.print ("ROBOT 1");
  }
  if (Report.Robot == 3){
    lcd.setCursor (5,0);
    lcd.print ("ROBOT 2");
  }
 // lcd_print (Report.Robot, Report.Message);
 delay(1000);
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

void ReceiveReport(){       //Function to receive data
  if(radio.available()){
    radio.read(&Report, sizeof(DataStruct));
  }
}

/*void lcd_print (int a, int b){
  if (Report.Robot == 0){
    lcd.setCursor (5,0);
    lcd.print ("UNBALL");
  }
  if (Report.Robot == 1){
    lcd.setCursor (5,0);
    lcd.print ("ROBOT 0");
  }
  if (a == 2){
    lcd.setCursor (5,0);
    lcd.print ("ROBOT 1");
  }
   if (a == 3){
    lcd.setCursor (5,0);
    lcd.print ("ROBOT 2");
  }
}*/




