#include "RF24.h"
#include <ros.h>
#include <communication_node/comm_msg.h>

/* Hardware configuration: Set up nRF24L01 radio on SPI protocol*/
RF24 radio(3, 2);   //arduino nano
//RF24 radio(A1, A0); //arduino pro micro
/**********************************************************/

byte pipe[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node", "7Node", "8Node"};
int msg_from_robot[2];
int msg_from_ROS[3];

int commands[2] = {0, 0};
int robotPipe = 0;

bool robot_online = false;


ros::NodeHandle nh;
communication_node::comm_msg msg;
communication_node::comm_msg to_pub;
ros::Publisher pubs("arduino_topic", &to_pub);

void messageCb( const communication_node::comm_msg& toggle_msg){
   msg = toggle_msg;
}

ros::Subscriber<communication_node::comm_msg> sub("radio_topic", &motorVel );

void setup() {
  Serial.begin(19200);
    while(!Serial);
  Serial.println("Inicio - central");
  radio.begin();

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  radio.openReadingPipe(1, pipe[1]); //n1->central pelo pipe0
  radio.openReadingPipe(2, pipe[5]); //n1->central pelo pipe0
  radio.openReadingPipe(3, pipe[3]); //n1->central pelo pipe0
  radio.openReadingPipe(4, pipe[7]); //n1->central pelo pipe0
  radio.startListening();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pubs);
}

void checkIfRadioIsPVariant() {
  to_pub.ispvariant = radio.isPVariant();
  if (radio.isPVariant()) {
    digitalWrite(13, HIGH); 
    Serial.println("versao nrf24l01+");
  }
}

bool receiveDataFromAnotherRadio() {
  if (radio.available()) {
    while (radio.available()) {
      radio.read(&msg_from_robot, sizeof(msg_from_robot));
    }
    return true;
  }
  return false;
}

void sendDataToSerialPort() {
  Serial.println(msg_from_robot[0]); //Serial.write() may be a better choice for this
  Serial.println(msg_from_robot[1]); //Serial.write() may be a better choice for this
}



void parseSerialMessageToRobot() {
  robotPipe = msg.data[0];
  commands[0] = msg.data[1];
  commands[1] = msg.data[2];
}

void sendDataToRobot() {
  radio.stopListening();
  radio.openWritingPipe(pipe[robotPipe]);
  robot_online = radio.write(commands, sizeof(commands));
  //Serial.println(robot_online);
  radio.startListening();
}

void loop() {
  to_pub = msg;
  nh.spinOnce();
  //if (receiveDataFromAnotherRadio())
  //  sendDataToSerialPort();
  //sendDataToRobot();
  checkIfRadioIsPVariant();
  pubs.publish(&to_pub);
  delay(1);
  delay(10);
}

