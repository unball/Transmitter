#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h> 
#include <stdio.h>
#include "config.hpp"

int16_t kpint;
int16_t kiint;
int16_t kdint;
int16_t wint;
int16_t vint;
double kpfloat;
double kifloat;
double kdfloat;
double wfloat;
double vfloat;
float erro;
char erro_buffer[50];
char serialData[50];
bool response = false;


snd_message send_commands;

rcv_message rcv_commands;

/* Declaração das funções */
void wifiSetup();
void sendWifi();
void sendConfig();
void receiveUSBdata();

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

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
  memcpy(&rcv_commands, incomingData, sizeof(rcv_commands));
  //Serial.println(rcv_commands.value);
}

void roboResponse(bool* response, float* erro){
  *erro = rcv_commands.value;
  *response = rcv_commands.response;
}

#if PID_TUNNER
// Callback function, execute when message is sent via Wi-Fi

#endif

/* Loop de setup */
void setup(void) {
  Serial.begin(115200);
  while(!Serial);
  wifiSetup();
  while (!response){
    sendConfig();
    roboResponse(&response, &erro);
    Serial.print("Não foi dessa vez amiguinho, tente mais tarde principalmente durante o programa");
    Serial.println(response);
    //response = true;
  }
  
  
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Loop que é executado continuamente */
void loop(){
    // Recebe robot_message via USB
    #if PID_TUNNER
    //receiveUSBdata();
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
    #endif
    

}

void sendConfig(){

  result = true;
  for(uint8_t i=0; i<3; i++){
    
    #if  CONTROL_ON
      snd_message config = {.control = 1, .id = i, .kp = 1, .ki =0,.kd = 0, .w = 0, .v = 0};

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

void sendWifi(){

  result = true;
  //parametro = twidle(parametro);
  for(uint8_t i=0; i<1; i++){
    /*kpint = (int16_t)(kpfloat * 100);
    kiint = (int16_t)(kifloat * 100);
    kdint = (int16_t)(kdfloat * 100);
    wint = (int16_t)(wfloat * 100);
    vint = (int16_t)(vfloat * 100);*/
    kpint = (int16_t)(0.159521 * 100);
    kiint = (int16_t)(0.016864 * 100);
    kdint = (int16_t)(0.016686 * 100);
    wint = (int16_t)(25 * 100);
    vint = (int16_t)(0 * 100);
    snd_message control_constants = {.id = i, .kp = kpint, .ki = kiint, .kd = kdint, .w = wint, .v = vint};
    /* Sends the message using ESP-NOW */
    esp_now_send(broadcastAddress, (uint8_t *) &control_constants, sizeof(snd_message));
    delay(3);
  }
  
  if(result){
    lastOK = millis();
  }
}

/*void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len){
        memcpy(&temp_msg, incomingData, sizeof(msg));
	    lastReceived = micros();
}*/

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
        sscanf(serialBuffer, "%lf %lf %lf", &kdfloat, &kifloat, &kdfloat);

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