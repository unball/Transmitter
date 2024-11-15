#include <Arduino.h>
#include <cstdint>
#include <sstream>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <iostream>
#include <string>

#define DEBUG 0

uint8_t deviceAddress[3][6] = { {0xCC,0x8D,0xA2,0x8D,0x0D,0x7C},
                                {0xCC,0x8D,0xA2,0x8B,0xD1,0x36},
                                {0x80,0x65,0x99,0xFC,0x40,0xCC} };

/* Estrutura para a mensagem a ser transmitida para o robô via Wi-Fi */
struct RobotMessage {
  int16_t vx;
  int16_t vy;
  int16_t w;
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
void detectKeyPressAndSend();

/* Contagem de erros de transmissão via USB detectados */
uint32_t erros = 0;
uint32_t lastOK = 0;
bool ackFlag = false;

// Variáveis para velocidades de movimento
float x_dot = 0.0;   // Velocidade na direção x
float y_dot = 0.0;   // Velocidade na direção y
float omega_dot = 0.0;  // Velocidade angular

// Callback when data is sent
#if DEBUG
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ackFlag = (status == ESP_NOW_SEND_SUCCESS);
}
#endif

/* Loop de setup */
void setup() {
  Serial.begin(115200);
  wifiSetup();
  pinMode(LED_BUILTIN, OUTPUT);
}

/* Loop que é executado continuamente */
void loop() {
    // Recebe robot_message via USB
    receiveUSBdata();

    // Envia via Wi-Fi
    static int32_t t = micros();
    if (micros() - t >= 100) {
        t = micros();
        sendWifi();
    }

    // Acende o LED se recebeu mensagem do USB em menos de 40ms
    if (millis() - lastOK < (DEBUG ? 40 : 5)) {
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }

    // Detecta a tecla pressionada e envia o comando correspondente
    detectKeyPressAndSend();
}

void detectKeyPressAndSend() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Lê até uma nova linha ou ENTER

        input.trim();  // Remove espaços e quebras de linha extras

        // Interpreta o comando
        if (input == "W" || "w") {  // Frente
            robot_message.vx = 0;
            robot_message.vy = 1;
            robot_message.w = 0;
            Serial.println("Comando: Frente");
        } else if (input == "A" || "a") {  // Esquerda
            robot_message.vx = -1;
            robot_message.vy = 0;
            robot_message.w = -0.5;
            Serial.println("Comando: Esquerda");
        } else if (input == "S" || "s") {  // Ré
            robot_message.vx = 0;
            robot_message.vy = -1;
            robot_message.w = 0;
            Serial.println("Comando: Ré");
        } else if (input == "D" || "d") {  // Direita
            robot_message.vx = 1;
            robot_message.vy = 0;
            robot_message.w = 0;
            Serial.println("Comando: Direita");
        } else {
            Serial.println("Comando desconhecido");
        }

        // Envia o comando atualizado para o robô
        sendWifi();
    }
}

/* Sends the message via Wi-Fi */
void sendWifi() {
    for (uint8_t i = 0; i < 3; i++) {
        int32_t checksum = robot_message.vx + robot_message.vy + robot_message.w;
        int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));

        std::stringstream parser;
        parser << '[' << (short int)i << ',' << robot_message.vx << ',' << robot_message.vy << ',' << robot_message.w << limitedChecksum << ']' << '\0';

        esp_err_t sendResult = esp_now_send(deviceAddress[i], (uint8_t *)(parser.str()).c_str(), (parser.str()).size());

        if (sendResult == ESP_OK) {
            #if DEBUG
            constexpr unsigned long TIMEOUT_MS = 30;
            unsigned long startTime = millis();

            while (!ackFlag && (millis() - startTime) < TIMEOUT_MS) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            if (ackFlag) {
                Serial.print("Mensagem recebida e ACK recebido do robô ");
                Serial.println(i);
                lastOK = millis();
            } else {
                Serial.print("Falha ao enviar ou não foi recebido o ACK para o robô ");
                Serial.println(i);
            }
            ackFlag = false;
            #else
            lastOK = millis();
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
}

/* Setup the Wi-Fi */
void wifiSetup() { 
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar o ESP-NOW");
        return;
  }

  #if DEBUG
  esp_now_register_send_cb(OnDataSent);
  #endif

  for (uint8_t i = 0; i < 3; i++) {
    memcpy(peerInfo.peer_addr, deviceAddress[i], 6);
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {  
        Serial.println("Erro ao adicionar o peer");
        return;
    }
  }

  esp_wifi_set_max_tx_power(20);
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
}

/* Reads new robot_message from serial */
void receiveUSBdata() {
    int initCounter = 0;

    while (Serial.available()) {
        char character = Serial.read();

        if (character == 'B') initCounter++;

        if (initCounter >= 3) {
            SerialMessage receive;
            Serial.readBytes((char *)(&receive), (size_t)sizeof(SerialMessage));

            int32_t checksum = 0;
            checksum += receive.data.vx + receive.data.vy + receive.data.w;

            int16_t limitedChecksum = checksum >= 0 ? (int16_t)(abs(checksum % 32767)) : -(int16_t)(abs(checksum % 32767));

            if (limitedChecksum == receive.checksum) {
                robot_message = receive.data;
                Serial.printf("%d\t%d\t%d\n", limitedChecksum, robot_message.vx, robot_message.vy, robot_message.w);
            } else {
                for (uint16_t i = 0; i < sizeof(SerialMessage); i++) {
                    Serial.printf("%p ", ((char *)&receive)[i]);
                }
            }
            initCounter = 0;
        }
    }
}
