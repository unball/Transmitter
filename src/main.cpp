#include <SPI.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define PID_TUNNER false

/* Definitions */
#define NUMBER_OF_ROBOTS 3
#define MAX_POWER 10.5  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values

/* Broadcast address, sends to everyone */
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Estrutura para a mensagem a ser transmitida para o robô via wi-fi */
struct Velocities{
  bool control;
  double v[3];
  double w[3];
};

#if PID_TUNNER
struct snd_message{
  uint8_t id;
  float kp;
  float ki;
  float kd;
};

snd_message send_commands;

struct rcv_message{
  double value;
};

rcv_message rcv_commands;

/* Struct for the message to be received from USB */
struct SerialConstants {
  snd_message data;
  double checksum;
};

#else
struct snd_message{
  bool control;
  uint8_t id;
  double vl;
  double vr;
};
#endif

/* Estrutura para a mensagem a ser recebida do USB */
struct SerialVelocities {
  bool control;
  Velocities data;
  double checksum;
};


/* Declaração das funções */
void wifiSetup();
void sendWifi();
void receiveUSBdata();

/* Mensagem a ser transmitida */
Velocities velocities;

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
    // Recebe velocities via USB
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

  for(uint8_t i=0 ; i<NUMBER_OF_ROBOTS ; i++){
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

/* Reads new velocities from serial */
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
      double checksum = 0;
      checksum = receive.data.kp + receive.data.ki + receive.data.kp;
      

      /* Verifies o checksum */
      if(checksum == receive.checksum){
        /* Copies to the global velocity buffer */
        send_commands = receive.data;
        // velocities = receive.data;
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

  for(uint8_t i=0 ; i<NUMBER_OF_ROBOTS ; i++){
    snd_message vel = {.control = velocities.control, .id = i, .vl = velocities.v[i], .vr = velocities.w[i]};
    
    /* Sends the message using ESP-NOW */
    esp_now_send(broadcastAddress, (uint8_t *) &vel, sizeof(snd_message));
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
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, WIFI_CHANNEL, NULL, 0);
}

/* Reads new velocities from serial */
void receiveUSBdata(){
  int initCounter = 0;
    
  while(Serial.available()){
    /* Lê um caracter da mensagem */
    char character = Serial.read();

    /* Incrementa o contador se for 'B' */
    if(character == 'B') initCounter++;

    /* Se os três primeiros caracteres são 'B' então de fato é o início da mensagem */
    if(initCounter >= 3){
      SerialVelocities receber;
      
      /* Lê a mensagem até o caracter de terminação e a decodifica */
      Serial.readBytes((char*)(&receber), (size_t)sizeof(SerialVelocities));

      /* Faz o checksum */
      double checksum = 0;
      for(int i=0 ; i<3 ; i++){
        checksum += receber.data.v[i] + receber.data.w[i];
      }

      /* Verifica o checksum */
      if(checksum == receber.checksum){
        /* Copia para o buffer global de velocities */
        velocities = receber.data;

        /* Reporta que deu certo */
        Serial.printf("%f\t%f\t%f\n", checksum, velocities.v[0], velocities.w[0]);
        
      }
      else {
        /* Devolve o checksum calculado se deu errado */
        for(uint16_t i=0 ; i<sizeof(SerialVelocities) ; i++){
          Serial.printf("%p ", ((char*)&receber)[i]);
        }
        Serial.println("");
        //Serial.printf("%p\t%p\t%p\n", checksum, velocities.v[0], velocities.w[0]);
      }

      /* Zera o contador */
      initCounter = 0;
    }
  }
}

#endif