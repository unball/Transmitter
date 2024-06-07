#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define PID_TUNNER false

/* Definitions */
/*
#define MAX_POWWIFI_POWER_19_5dBmER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values */

/* Broadcast address, sends to everyone */
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct RobotMessage{
  int16_t v[3];
  int16_t w[3];
};

#if PID_TUNNER
struct snd_message{
  uint8_t id;
  int16_t kp;
  int16_t ki;
  int16_t kd;
};

snd_message send_commands;

struct rcv_message{
  float value;
};

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

#if PID_TUNNER
// Callback function, execute when message is sent via Wi-Fi
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  Serial.println(rcv_commands.value);
  delay(2);
}
#endif

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

#if PID_TUNNER
/* Sends the message via Wi-Fi */
void sendWifi(){

  result = true;

  for(uint8_t i=0 ; i<3 ; i++){
    snd_message control_constants = {.id = i, .kp = 1.0, .ki = 1.0, .kd = 1.0};
    
    /* Sends the message using ESP-NOW */
    esp_now_send(broadcastAddress[i], (uint8_t *) &control_constants, sizeof(snd_message));
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
  int initCounter = 0;
    
  while(Serial.available()){
    /* Reads a character from the message */
    char character = Serial.read();

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

  /*Sets WiFi max output*/
  /*WiFi.setTxPower(WIFI_POWER_19_5dBm); Set max power replacement can be done with this and the define in the beggining of the file*/

  /* Initialize the ESP-NOW */
  if (esp_now_init() != 0) {
    return;
  }
  /*Set as wireless (Neither CONTROLLER nor SLAVE are present in the esp32 library)
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent); */ 
  
  /*Starts Netif*/
  esp_netif_init(); /*ESP_ERROR_CHECK can also be added in every function here in wifiSetup()*/

  /*Loop*/
  esp_event_loop_create_default();

  /*Initialize the WiFi*/
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  /*Sets storage*/
  esp_wifi_set_storage(WIFI_STORAGE_RAM);

  /* Puts the device in Wi-Fi Station mode */
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();

  /*Set channel*/
  esp_wifi_set_channel(12, WIFI_SECOND_CHAN_NONE);

  /*Set max power*/
  esp_wifi_set_max_tx_power(10);

  /* Registers the receiver of the message 
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0); Set channel function can be replaced with this and the define in the beggining of the file*/
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
      int16_t checksum = 0;
      for(int i=0 ; i<3 ; i++){
        checksum += receive.data.v[i] + receive.data.w[i];
      }

      /* Verifica o checksum */
      if(checksum == receive.checksum){
        /* Copia para o buffer global de robot_message */
        robot_message = receive.data;

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
      initCounter = 0;
    }
  }
}

#endif