#include <Arduino.h>
#include <cstdint>
#include <sstream>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define DEBUG 0

uint8_t deviceAddress[3][6] = { {0xCC,0x8D,0xA2,0x8D,0x0D,0xD4},
                                {0xCC,0x8D,0xA2,0x8D,0x0D,0x02},
                                {0xCC,0x8D,0xA2,0x8B,0xD0,0x2A} };

/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct RobotMessage{
  int16_t v[3];
  int16_t w[3];
};

/* Estrutura para a mensagem a ser recebida do USB */
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
bool ackFlag=false;

//Callback when data is sent
#if DEBUG
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ackFlag = (status == ESP_NOW_SEND_SUCCESS) ;
}
#endif

/* Loop de setup */
void setup() {
  Serial.begin(115200);
  wifiSetup();
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Loop que é executado continuamente */
void loop(){
    // Recebe robot_message via USB
    receiveUSBdata();

    // Envia via rádio
		// static int32_t t = micros();
		// if(micros()-t >= 10){
			// t = micros();
    sendWifi();
		// }

    // Acende o LED se recebeu mensagem do USB em menos de 40ms
    //Serial.print("latencia de: "); Serial.println(millis()-lastOK); //ms
    // Sabe-se que a latência média de transmissão é de 36ms
    if(millis()-lastOK < (DEBUG?40:5) ){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
    }

}

/* Sends the message via Wi-Fi */
void sendWifi(){
  for(uint8_t i=0 ; i<3 ; i++){

    int32_t checksum = robot_message.v[i] + robot_message.w[i];
    int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));
    // padrão da mensagem: "[id,v,w,checksum]"
    std::stringstream parser;
    parser << '[' << (short int)i << ',' << robot_message.v[i] << ',' << robot_message.w[i] << ',' << limitedChecksum << ']' << '\0';     //printf("%s\n",(parser.str()).c_str());

    esp_err_t sendResult = esp_now_send(deviceAddress[i], (uint8_t *) (parser.str()).c_str(), (parser.str()).size());

    if (sendResult == ESP_OK) {

    #if DEBUG
      constexpr unsigned long TIMEOUT_MS = 30;  // Timeout para o ACK. Caso necessário, mude aqui || use const caso tenha erros de compilação
      unsigned long startTime = millis();

      while (!ackFlag && (millis() - startTime) < TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      if (ackFlag) {
        // ACK recebido
        Serial.print("Mensagem recebida e ACK recebido do robô ");
        Serial.println(i);
        lastOK=millis();
      } else{
        // ACK não recebido no tempo estabelecido
        Serial.print("Falha ao enviar ou não foi recebido o ACK para o robô ");
        Serial.println(i);
      }
      ackFlag=false;
    #else
      lastOK=millis();
    #endif
    }
    vTaskDelay(pdMS_TO_TICKS(0.01));
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

  #if DEBUG
  esp_now_register_send_cb(OnDataSent);
  #endif

  for(uint8_t i=0 ; i<3 ; i++){

  esp_now_peer_info_t peerInfo;

  memcpy(peerInfo.peer_addr, deviceAddress[i], 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  
      Serial.println("Erro ao adicionar o peer");
      return;
    }
  }

  esp_err_t error = esp_wifi_set_max_tx_power(20);

  if (error != ESP_OK){
    return;
  }
   esp_err_t error2 = esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);

   if (error2 != ESP_OK){
     return;
   }
}

struct __attribute__((packed)) SerialMessage {
  int16_t v[3];       
  int16_t w[3];       
  int16_t checksum;   
};

SerialMessage received_message;


void receiveUSBdata(){

  if (Serial.available() < sizeof(SerialMessage)){
  return;
  }


  if (Serial.read() == 'B' && Serial.read() == 'B' && Serial.read() == 'B') {

    Serial.readBytes((char*)&received_message, sizeof(SerialMessage));
    int32_t checksum = 0;
    for(int i=0 ; i<3 ; i++){
        checksum += received_message.v[i] + received_message.w[i];
      }
    int16_t limitedChecksum = (checksum >= 0 ? 1 : -1) * (abs(checksum) % 32767);
  if (limitedChecksum == received_message.checksum) {
    memcpy(robot_message.v,received_message.v,sizeof(received_message.v));
    memcpy(robot_message.w,received_message.w,sizeof(received_message.w));
    
    
    lastOK = millis();
  }
  }else{
    Serial.read();
  }
    #if DEBUG
    Serial.print("Mensagem recebida: ");
    Serial.println(message);
    #endif

    lastOK = millis();
}