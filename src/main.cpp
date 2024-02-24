#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h> 
#include <stdio.h>
#include "config.hpp"

// int16_t kp;
// int16_t ki;
// int16_t kd;
float erro;
char erro_buffer[50];
char serialData[50];
bool response = false;

enum Mode {
  twiddle = 0,
  control = 1,
  no_control = 2,
};

uint8_t mode;

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

snd_message send_commands;

rcv_message rcv_commands;

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
  //Serial.println(rcv_commands.value);
  delay(2);
}

/*void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
        memcpy(&temp_msg, incomingData, sizeof(msg));
	    lastReceived = micros();
}*/

void robotResponse(bool* response, float* erro){
  *erro = rcv_commands.value;
  *response = rcv_commands.response;
}

/* Loop de setup */
void setup(void) {
  Serial.begin(115200);
  while(!Serial);
  wifiSetup();
  while (!response){
    sendConfig();
    robotResponse(&response, &erro);
    Serial.println(response);
  }
  
  
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

    dtostrf(erro, 8, 5, erro_buffer);
    erro = rcv_commands.value; 
    Serial.write(erro_buffer,4);    

}

void sendConfig(){

  result = true;
  for(uint8_t i=0; i<3; i++){
    
    #if  CONTROL_ON
      snd_message config = {.control = 1, .id = i, .kp = 1, .ki =0,.kd = 0, .v = 0, .w = 0};

    #else
      #if TWIDDLE_ON
      snd_message config = {.control = 2, .id = i, .kp = 1, .ki =0,.kd = 0, .w = 0, .v = 0};
      
      #else
      snd_message config = {.control = 3, .id = i, .kp = 1, .ki =0,.kd = 0, .w = 0, .v = 0};

      #endif
    #endif
   
    esp_now_send(broadcastAddress, (uint8_t *) &config, sizeof(snd_message));
    delay(3);
  }
  
  if(result){
    lastOK = millis();
  }
}

/* Sends the message via Wi-Fi */
void sendWifi(){

  result = true;
  
  for(uint8_t i=0; i<3; i++){
    // TODO: Fazer com que de alguma forma, após a execução do twiddle, o código armazene as constantes kp, ki e kd pra utilizar no modo de controle.
    // kp = (int16_t)(0.159521 * 100);
    // ki = (int16_t)(0.016864 * 100);
    // kd = (int16_t)(0.016686 * 100);
    
    snd_message control_constants = {.id = i, .kp = 0, .ki = 0, .kd = 0, .v = robot_message.v[i], .w = robot_message.w[i]};

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
void receiveUSBdataAntigo(){
  int counter_no_control = 0;
  int counter_control = 0;
    
  while(Serial.available()){
    /* Lê um caracter da mensagem */
    char character = Serial.read();

    /* Incrementa o contador para rotina sem controle */
    if(character == 'B') counter_no_control++;

    /* Incrementa o contador para rotina com controle */
    if(character == 'C') counter_control++;

    /* Se os três primeiros caracteres são 'B' então é o início da mensagem sem controle*/
    if(counter_no_control >= 3){
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

    /* Se os três primeiros caracteres são 'C' então é o início da mensagem com controle*/
    if(counter_control >= 3){
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
        mode = Mode::control;

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
   
  }
}

void receiveUSBdata() {
  static char serialBuffer[50]; // Buffer para armazenar os caracteres lidos
  static int bufferIndex = 0; // Índice atual no buffer
  static bool messageStart = false; // Flag para indicar se o início da mensagem foi encontrado

  // Lê caracteres da porta serial até que a linha seja completa
  while (Serial.available() > 0) {
    char character = Serial.read();

    // Se encontrar o caractere 'T', incrementa o contador de início da mensagem
    if (character == 'T') {
      if (!messageStart) {
        messageStart = true;
        bufferIndex = 0;
      }
    }

    // Se estiver dentro da mensagem
    if (messageStart) {
      // Se encontrar o caractere de nova linha, a mensagem está completa
      if (character == '\n') {
        // Adiciona terminador de string ao buffer
        serialBuffer[bufferIndex] = '\0';

        // Tenta analisar a linha para obter os três valores de ponto flutuante
        // sscanf(serialBuffer, "%lf %lf %lf", &kdfloat, &kifloat, &kdfloat);

        // Reinicia as variáveis para a próxima leitura
        messageStart = false;
        bufferIndex = 0;
      } else {
        // Adiciona o caractere ao buffer
        serialBuffer[bufferIndex++] = character;
      }
    }
  }
}