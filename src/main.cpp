#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h> 
#include <stdio.h>
#include "config.hpp"

ControlConstants control_constants;

/* Definitions */
#define MAX_POWER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values

/* Broadcast address, sends to everyone */
uint8_t broadcastAddress[3][7] = {{0xC8, 0xC9, 0xA3, 0x3A, 0xD9, 0xE5},
                                  {0x5C, 0xCF, 0x7F, 0xDE, 0x1D, 0x36},
                                  {0x2C, 0x3A, 0xE8, 0x00, 0xDE, 0xA0}};

/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct RobotMessage{
  int16_t v[3];
  int16_t w[3];
};

uint8_t mode;

snd_message send_commands;

rcv_message rcv_commands;

/* Struct for the message to be received from USB */
struct SerialConstants {
  snd_message data;
  double checksum;
};

#else
struct snd_message{
  int16_t id;
  int16_t v;
  int16_t w;
  int16_t checksum;
};
#endif

/* Estrutura para a mensagem a ser recebida do USB */
struct SerialMessage {
  RobotMessage data;
  int16_t checksum;
};

/* Declaração das funções */
void wifiSetup();
void sendWifi();
void sendConfig();
void receiveUSBdata();

/* Mensagem a ser transmitida */
RobotMessage robot_message;

/* Contagem de erros de transmissão via USB detectados */
uint32_t erros = 0;
uint32_t lastOK = 0;
bool result;

//Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  if (sendStatus == 0)
    result = true;
  else
    result = false;
}

// Callback function, execute when message is sent via Wi-Fi
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  // TODO: O que fazer com id? verificar se está no robô certo?
  Serial.println(rcv_commands.error/100);
  delay(2);
}

/* Loop de setup */
void setup(void) {
  Serial.begin(115200);
  while(!Serial);
  wifiSetup();
  
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Loop que é executado continuamente */
void loop(){
    // Recebe robot_message via USB
    receiveUSBdata();

    // Envia via rádio
		static int32_t t = micros();
		if(micros()-t >= 500){
			t = micros();
      sendWifi();
		}

    // Acende o LED se recebeu mensagem do USB em menos de 5ms
    if(millis()-lastOK < 5){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
    }

}

/* Sends the message via Wi-Fi */
void sendWifi(){

  result = true;
  
  if (mode == Mode::twiddle){
    /* Sends the message using ESP-NOW */
    esp_now_send(broadcastAddress[i], (uint8_t *) &control_constants, sizeof(snd_message));
    delay(3);
  }
  else{
    for(uint8_t i=0; i<3; i++){

      snd_message msg = {.control = mode, .id = i, .kp = control_constants.kp[i], .ki = control_constants.ki[i], .kd = control_constants.kd[i], .v = robot_message.v[i], .w = robot_message.w[i]};

      /* Sends the message using ESP-NOW */
      esp_now_send(broadcastAddress, (uint8_t *) &msg, sizeof(snd_message));
      delay(3);
    }
  }
  
  if(result){
    lastOK = millis();
  }
}

/* Setup the Wi-Fi  */
void wifiSetup(){
  /* Puts the device in Wi-Fi Station mode */
  WiFi.mode(WIFI_STA);

  WiFi.setOutputPower(MAX_POWER);

  /* Initialize the ESP-NOW */
  if (esp_now_init() != 0) {
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  /* Registers the receiver of the message */
  for(int i =0; i<3; i++){
    esp_now_add_peer(broadcastAddress[i], ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
  }

}

/* Reads new robot_message from serial */
void receiveUSBdata(){
  int counter_no_control = 0;
  int counter_control = 0;
  int counter_twiddle = 0;

    /* Increments the counter if it's 'T' */
    if(character == 'T') initCounter++;

    /* If the first characters are 'T', then in fact it is the beginning of the message */
    if(initCounter >= 3){
      SerialConstants receive;

      /* Reads the message until the finishing character then decode it */
      Serial.readBytes((char*)(&receive), (size_t)sizeof(SerialConstants));

      /* Does the checksum */
      int16_t checksum = 0;
      checksum = receive.data.kp + receive.data.ki + receive.data.kp;
      

      /* Verifies o checksum */
      if(checksum == receive.checksum){
        /* Copies to the global velocity buffer */
        send_commands = receive.data;
        // robot_message = receive.data;
      }

      /* Reset the counter */
      initCounter = 0;
    }
  }
}

#else

/* Sends the message via Wi-Fi */
void sendWifi(){

  result = true;

  for(uint8_t i=0 ; i<3 ; i++){
    
    int32_t checksum = robot_message.v[i] + robot_message.w[i];
    int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));

    snd_message msg = {.id = i, .v = robot_message.v[i], .w = robot_message.w[i], .checksum = limitedChecksum};
    
    /* Sends the message using ESP-NOW */
    esp_now_send(broadcastAddress[i], (uint8_t *) &msg, sizeof(snd_message));
    delay(3);
  }
  
  if(result){
    lastOK = millis();
  }
}

/* Setup the Wi-Fi  */
void wifiSetup(){
  /* Puts the device in Wi-Fi Station mode */
  WiFi.mode(WIFI_STA);

  WiFi.setOutputPower(MAX_POWER);

  /* Initialize the ESP-NOW */
  if (esp_now_init() != 0) {
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  
  /* Registers the receiver of the message */
  for(int i = 0; i < 3; i++){
    esp_now_add_peer(broadcastAddress[i], ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
  }
}

/* Reads new robot_message from serial */
void receiveUSBdata(){
  int initCounter = 0;
    
  while(Serial.available()){
    /* Lê um caracter da mensagem */
    char character = Serial.read();

    /* Incrementa o contador para rotina sem controle */
    if(character == 'B') counter_no_control++;

    /* Incrementa o contador para rotina com controle */
    if(character == 'C') counter_control++;

    /* Incrementa o contador para rotina do Twiddle */
    if(character == 'T') counter_twiddle++;

    /* Se os três primeiros caracteres são 'B' então é o início da mensagem sem controle */
    if(counter_no_control >= 3){
      SerialMessage receive;
      
      /* Lê a mensagem até o caracter de terminação e a decodifica */
      Serial.readBytes((char*)(&receive), (size_t)sizeof(SerialMessage));

      /* Faz o checksum */
      int32_t checksum = 0;
      for(int i=0 ; i<3 ; i++){
        checksum += receive.data.v[i] + receive.data.w[i];
      }

      int16_t limited_checksum = checksum >= 0 ? (int16_t)(abs(checksum) % 32767) : -(int16_t)(abs(checksum) % 32767);

      /* Verifica o checksum */
      if(limited_checksum == receive.checksum){
        /* Copia para o buffer global de robot_message */
        robot_message = receive.data;
        mode = Mode::no_control;

        /* Reporta que deu certo */
        Serial.printf("%d\t%d\t%d\n", checksum, robot_message.v[0], robot_message.w[0]);
        
      }
      else {
        /* Devolve o checksum calculado se deu errado */
        for(uint16_t i=0 ; i<sizeof(SerialMessage) ; i++){
          Serial.printf("%p ", ((char*)&receive)[i]);
        }
        Serial.println("");
        //Serial.printf("%p\t%p\t%p\n", checksum, robot_message.v[0], robot_message.w[0]);
      }

      /* Zera o contador */
      counter_no_control = 0;

    }

    /* Se os três primeiros caracteres são 'C' então é o início da mensagem com controle */
    if(counter_control >= 3){
      SerialMessage receive;
      
      /* Lê a mensagem até o caracter de terminação e a decodifica */
      Serial.readBytes((char*)(&receive), (size_t)sizeof(SerialMessage));

      /* Faz o checksum */
      int32_t checksum = 0;
      for(int i=0 ; i<3 ; i++){
        checksum += receive.data.v[i] + receive.data.w[i];
      }
      int16_t limited_checksum = checksum >= 0 ? (int16_t)(abs(checksum) % 32767) : -(int16_t)(abs(checksum) % 32767);

      /* Verifica o checksum */
      if(limited_checksum == receive.checksum){
        /* Copia para o buffer global de robot_message */
        robot_message = receive.data;
        mode = Mode::control;

        /* Reporta que deu certo */
        Serial.printf("%d\t%d\t%d\n", limited_checksum, robot_message.v[0], robot_message.w[0]);
        
      }
      else {
        /* Devolve o checksum calculado se deu errado */
        for(uint16_t i=0 ; i<sizeof(SerialMessage) ; i++){
          Serial.printf("%p ", ((char*)&receive)[i]);
        }
        Serial.println("");
        //Serial.printf("%p\t%p\t%p\n", checksum, robot_message.v[0], robot_message.w[0]);
      }

      /* Zera o contador */
      counter_control = 0;
      
    }
   
    /* Se os três primeiros caracteres são 'T' então é o início da mensagem para a rotina do Twiddle*/
    if(counter_twiddle >= 3){
      SerialConstants receive_constants;
      
      /* Lê a mensagem até o caracter de terminação e a decodifica */
      Serial.readBytes((char*)(&receive_constants), (size_t)sizeof(SerialConstants));

      mode = Mode::twiddle;

      serial_constants = receive_constants;

      /* Zera o contador */
      counter_twiddle = 0;
      
    }
  }

}


