#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 

#define PID_TUNNER false

/* Definitions */
#define MAX_POWER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values

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

uint8_t broadcastAddress[3][7] = {{0xA0,0xDD,0x6C,0x04,0x7E,0x0C},
                                  {0xC8, 0xC9, 0xA3, 0x3A, 0xDE, 0x9E},
                                  {0x2C, 0x3A, 0xE8, 0x00, 0xDE, 0xA0}};

/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct RobotMessage{
  int16_t vx;
  int16_t vy;
  int16_t w;
};

struct snd_message{
  int16_t id;
  int16_t vx;
  int16_t vy;
  int16_t w;
  int16_t checksum;
};

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
    // receiveUSBdata();
    detectKeyPressAndSend();

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


void detectKeyPressAndSend() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Lê até uma nova linha ou ENTER

        input.trim();  // Remove espaços e quebras de linha extras

        // Interpreta o comando
        if (input == "W" || "w") {  // Frente
            
            robot_message.vx = 1;
            robot_message.vy = 0;
            robot_message.w = 0;
            Serial.println("Comando: Frente");
        } else if (input == "A" || "a") {  // Esquerda
            robot_message.vx = 0;
            robot_message.vy = 1;
            robot_message.w = 0;
            Serial.println("Comando: Esquerda");
        } else if (input == "S" || "s") {  // Ré
            robot_message.vx = -1;
            robot_message.vy = 0;
            robot_message.w = 0;
            Serial.println("Comando: Ré");
        } else if (input == "D" || "d") {  // Direita
            robot_message.vx = 0;
            robot_message.vy = -1;
            robot_message.w = 0;
            Serial.println("Comando: Direita");
        } else {
            Serial.println("Para");
            robot_message.vx = 0;
            robot_message.vy = 0;
            robot_message.w = 0;
        }

        // Envia o comando atualizado para o robô
        // sendWifi();
    }
}

/* Sends the message via Wi-Fi */
void sendWifi(){

  result = true;

  for(uint8_t i=0 ; i<1 ; i++){
    
    int32_t checksum = robot_message.vx +  robot_message.vy + robot_message.w;
    int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));

    snd_message msg = {.id = i, .vx = robot_message.vx, .vy = robot_message.vy, .w = robot_message.w, .checksum = limitedChecksum};
    
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

  /* Initialize the ESP-NOW */
  if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar o ESP-NOW");
        return;
    }

  esp_now_peer_info_t peerInfo;

  memcpy(peerInfo.peer_addr, broadcastAddress[0], 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  
      Serial.println("Erro ao adicionar o peer");
      return;
    }

  esp_err_t error = esp_wifi_set_max_tx_power(20);
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
      checksum += receive.data.vx + receive.data.vy + receive.data.w;

      /* Verifica o checksum */
      if(checksum == receive.checksum){
        /* Copia para o buffer global de robot_message */
        robot_message = receive.data;

        /* Reporta que deu certo */
        Serial.printf("%d\t%d\t%d\n", checksum, robot_message.vx, robot_message.vy, robot_message.w);
        
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