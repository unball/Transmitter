#include <Arduino.h>
#include <sstream>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


/* Definitions */
/*
#define MAX_POWWIFI_POWER_19_5dBmER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values */

/* Broadcast address, sends to everyone */
// LISTA DE MAC DOS WEMOS8266
// a - {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
// b - {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
// c - {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
// 3 - {0x48,0x55,0x19,0xF6,0x44,0x44}
// 4 - {0x5C,0xCF,0x7F,0xDE,0x1D,0x36}
// 5 - {0x2C,0x3A,0xE8,0x00,0xDE,0xA0}
// 6 - {0xC8,0xC9,0xA3,0x3A,0xDE,0x9E}
// 7 - {0xB4,0xE6,0x2D,0x04,0x06,0x4F}
// 8 - {0x98,0xCD,0xAC,0x39,0x70,0xBC}

/* Broadcast address, sends to everyone */
// LISTA DE MAC DOS WEMOS32
// a - {0xCC,0x8D,0xA2,0x8C,0x31,0xB6}
// b - {0xCC,0x8D,0xA2,0x8B,0xCF,0xC8}

uint8_t broadcastAddress[3][6] = {{0xCC,0x8D,0xF2,0x6B,0xD0,0xCC},
                                  {0xCC,0x8D,0xA2,0x8B,0xD1,0x36},
                                  {0xCC,0x8D,0xF7,0x0B,0x81,0x36}};


/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct RobotMessage{
  int16_t v[3];
  int16_t w[3];
};

/* Estrutura para a mensagem a ser recebida do USB */
struct SerialMessage {
  RobotMessage data;
  int16_t checksum;
};

/* Mensagem a ser transmitida */
RobotMessage robot_message;

esp_now_peer_info_t peerInfo;

/* Declaração das funções */
void wifiSetup();
void sendWifi();
void receiveUSBdata();

/* Contagem de erros de transmissão via USB detectados */
uint32_t erros = 0;
uint32_t lastOK = 0;
bool result=false;

//Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS)
    result = true;
  else
    result = false;
}

/* Loop de setup */
void setup() {
  Serial.begin(115200);
  wifiSetup();
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Loop que é executado continuamente */
void loop(){
    // Recebe robot_message via USB
    //receiveUSBdata();

    // Envia via rádio
		static int32_t t = micros();
		if(micros()-t >= 100){
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
  for(uint8_t i=0 ; i<3 ; i++){

    result = true;  // ??
    int32_t checksum = robot_message.v[i] + robot_message.w[i];
    int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));

    // padrão da mensagem: "<id,v,w,checksum>"
    std::stringstream parser;
    parser << '<' << (short int)i << ',' << robot_message.v[i] << ',' << robot_message.w[i] << ',' << limitedChecksum << '>' << '\0'; 
    //printf("%s\n",(parser.str()).c_str());

    esp_err_t sendResult = esp_now_send(broadcastAddress[i], (uint8_t *) (parser.str()).c_str(), (parser.str()).size());
    if (sendResult == ESP_OK) {
    lastOK = millis();
    }

    delay(3);
  }
}

/* Setup the Wi-Fi  */
void wifiSetup(){ 
  /* Puts the device in Wi-Fi Station mode */
  WiFi.mode(WIFI_STA);

  /* Initialize the ESP-NOW */
  if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar o ESP-NOW");
        return;
    }

  esp_now_register_send_cb(OnDataSent);

  for(uint8_t i=0 ; i<3 ; i++){

  esp_now_peer_info_t peerInfo;

  memcpy(peerInfo.peer_addr, broadcastAddress[i], 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  
      Serial.println("Erro ao adicionar o peer");
      return;
    }
  }

  esp_err_t error = esp_wifi_set_max_tx_power(20);

  if (error == ESP_OK){
    return;
  }
   esp_err_t error2 = esp_wifi_set_channel(12, WIFI_SECOND_CHAN_NONE);

   if (error2 == ESP_OK){
     return;
   }
}

/* Reads new robot_message from serial */
void receiveUSBdata(){
  int initCounter = 0;
    
  while(Serial.available()){
    /* Lê um caracter da mensagem */
    char character = Serial.read();

    /* Incrementa o contador se for 'B' */
    if(character == 'B') initCounter++;

    /* Se os três primeiros caracteres são 'B' então de fato é o início da mensagem */
    if(initCounter >= 3){
      SerialMessage receive;
      
      /* Lê a mensagem até o caracter de terminação e a decodifica */
      Serial.readBytes((char*)(&receive), (size_t)sizeof(SerialMessage));

      /* Faz o checksum */
      int32_t checksum = 0;
      for(int i=0 ; i<3 ; i++){
        checksum += receive.data.v[i] + receive.data.w[i];
      }

      int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));

      /* Verifica o checksum */
      if(limitedChecksum == receive.checksum){
        /* Copia para o buffer global de robot_message */
        robot_message = receive.data;

        /* Reporta que deu certo */
        Serial.printf("%d\t%d\t%d\n", limitedChecksum, robot_message.v[0], robot_message.w[0]);
        
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
      initCounter = 0;
    }
  }
}

