#include <Arduino.h>
#include <TeensyThreads.h>
#include "Telem.h"

void hb_thd()
{

    SEND_HB(var_type, var_autopilot_type, var_system_mode, var_custom_mode, var_system_state);
}

void setup() {
  Serial.begin(115200);
  // start_time = millis();
  threads.addThread(hb_thd, 1);
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