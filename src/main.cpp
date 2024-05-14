#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define PID_TUNNER false

/* Definitions */
#define MAX_POWER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values

/* Broadcast address, sends to everyone */
uint8_t broadcastAddress[3][7] = {{0xC8, 0xC9, 0xA3, 0x3A, 0xD9, 0xE5},
                                  {0x5C, 0xCF, 0x7F, 0xDE, 0x1D, 0x36},
                                  {0x2C, 0x3A, 0xE8, 0x00, 0xDE, 0xA0}};

/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct RobotMessage{
  int16_t vl[3];
  int16_t vr[3];
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
  int16_t vl;
  int16_t vr;
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
    esp_now_send(broadcastAddress, (uint8_t *) &control_constants, sizeof(snd_message));
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
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);

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
    snd_message msg = {.id = i, .vl = robot_message.vl[i], .vr = robot_message.vr[i]};
    
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
        checksum += receive.data.vl[i] + receive.data.vr[i];
      }

      /* Verifica o checksum */
      if(checksum == receive.checksum){
        /* Copia para o buffer global de robot_message */
        robot_message = receive.data;

        /* Reporta que deu certo */
        Serial.printf("%d\t%d\t%d\n", checksum, robot_message.vl[0], robot_message.vr[0]);
        
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