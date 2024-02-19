#include <Arduino.h>
#include "common/mavlink.h"


int sysid = 1;
int compid = 158;
int type = MAV_TYPE_QUADROTOR;
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
// mavlink_message_t msg;
// mavlink_heartbeat_t hb;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t system_mode = MAV_MODE_MANUAL_ARMED; /// /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
uint32_t custom_mode = MAV_MODE_FLAG_GUIDED_ENABLED; ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY;
unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000;

void SEND_HB();
void SEND_PARAMS();
void SEND_AHRS();

void setup() {
  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) 
  {
    SEND_HB();
    previousMillisMAVLink = currentMillisMAVLink;
  }
}

// put function definitions here:
void SEND_HB() /// heartbeat established
{
  mavlink_message_t msg;
  mavlink_heartbeat_t hb;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(255,1, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_encode(255,1, &msg, &hb );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void SEND_PARAMS(){}
void SEND_AHRS(){}
