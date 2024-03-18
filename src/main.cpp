#include <Arduino.h>
#include <TeensyThreads.h>
#include "Telem.h"

#define PLACEHOLDER_PARAM_ID 100
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

float placeholder_param = 42.0; // Placeholder value

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

uint64_t start_time; // Variable to store the start time

void SEND_HB();
void SEND_PARAMS();
void SEND_AHRS();
void SEND_GPS();
void TESTSEND();

// put function definitions here:
void SEND_HB() /// heartbeat established
{
  mavlink_message_t msghb;
  mavlink_heartbeat_t hb;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(255,1, &msghb, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_encode(255,1, &msghb, &hb );
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msghb);
  Serial.write(buf, len);
}

void SEND_PARAMS()
{
  mavlink_message_t msgprm;
  // mavlink_param_request_list_t param_req;
  // mavlink_param_request_read_t param_red;
  mavlink_param_value_t param_msg;

  param_msg.param_value = 16;
  param_msg.param_count = 1;
  param_msg.param_index = 0;
  strcpy(param_msg.param_id, "MY_PARAM");


  mavlink_msg_param_value_encode(255,1,&msgprm,&param_msg);
  
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msgprm);

 Serial.write(buf, len);
}

void SEND_AHRS()
{
 mavlink_message_t msgahrs;
 mavlink_attitude_t attitude_msg;
 attitude_msg.roll = random(-20,20);
 attitude_msg.rollspeed = 0;
 attitude_msg.pitch = 0;
 attitude_msg.pitchspeed = 0;
 attitude_msg.yaw = 0;
 attitude_msg.yawspeed = 0;

  mavlink_msg_attitude_encode(255,1, &msgahrs, &attitude_msg);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msgahrs);

 Serial.write(buf, len);

}

void SEND_GPS(uint64_t time)
{
  mavlink_message_t msggps;
  mavlink_gps2_raw_t gps_msg;

 gps_msg.time_usec = time; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 gps_msg.lat = -7.767218; /*< [degE7] Latitude (WGS84)*/
 gps_msg.lon = 110.233244; /*< [degE7] Longitude (WGS84)*/
 gps_msg.alt = 20000; /*< [mm] Altitude (MSL). Positive for up.*/
 gps_msg.dgps_age = 1; /*< [ms] Age of DGPS info*/
 gps_msg.eph = UINT16_MAX; /*<  GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
 gps_msg.epv = UINT16_MAX; /*<  GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
 gps_msg.vel = UINT16_MAX; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
 gps_msg.cog = UINT16_MAX; /*< [cdeg] Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 gps_msg.fix_type =  GPS_FIX_TYPE_3D_FIX; /*<  GPS fix type.*/
 gps_msg.satellites_visible = UINT8_MAX; /*<  Number of satellites visible. If unknown, set to UINT8_MAX*/
 gps_msg.dgps_numch = 0; /*<  Number of DGPS satellites*/
 gps_msg.yaw = 36000; /*< [cdeg] Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.*/
 gps_msg.alt_ellipsoid = 20000; /*< [mm] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.*/
 gps_msg.h_acc = 1000; /*< [mm] Position uncertainty.*/
 gps_msg.v_acc = 1000; /*< [mm] Altitude uncertainty.*/
 gps_msg.vel_acc = 1000; /*< [mm] Speed uncertainty.*/
 gps_msg.hdg_acc = 10; /*< [degE5] Heading / track uncertainty*/

  mavlink_msg_gps2_raw_encode(255,1,&msggps, &gps_msg);
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msggps);

  Serial.write(buf, len);
}

void gps_thd()
{
  while(1)
  {  
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) 
  {
    SEND_HB();
    previousMillisMAVLink = currentMillisMAVLink;
  }
  // SEND_HB();
    threads.yield();
}
}

void setup() {
  Serial.begin(115200);
  // start_time = millis();
  threads.addThread(gps_thd, 1);
}

void loop() {
  // Update timestamp
  // uint64_t current_time = millis();
  // uint64_t protocoltime = (current_time - start_time) * 1000ULL; // Convert to microseconds


  // unsigned long currentMillisMAVLink = millis();
  // if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) 
  // {
  //   SEND_HB();
  //   previousMillisMAVLink = currentMillisMAVLink;
  // }
  SEND_PARAMS();
  // SEND_AHRS();
  // SEND_GPS(protocoltime);
}