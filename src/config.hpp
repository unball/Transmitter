#include <cstdint>

#define PID_TUNNER true
#define MAX_POWER 10.0  //TODO: Test values
#define WIFI_CHANNEL 12  //TODO: Test values
#define TWIDDLE_ON false
#define CONTROL_ON true



/* Broadcast address, sends to everyone */
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
struct snd_message{
  uint8_t control;  
  uint8_t id;
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t w;
  int16_t v;
};


struct rcv_message{
  uint8_t id;  
  bool response;
  int16_t value;
};

struct SerialConstants {
  snd_message data;
  double checksum;
};
