#ifndef CAN_RX_H
#define CAN_RX_H

#include <Arduino.h>
#include "driver/twai.h"   // TWAI driver for ESP32 CAN

// Struct to store both sensors’ readings
struct LidarData {
  // ---- Raw readings from CAN (unfiltered) ----
  uint16_t rawDistA;
  uint16_t rawStrA;
  uint16_t rawDistB;
  uint16_t rawStrB;

  // ---- Filtered values ----
  uint16_t distA;
  uint16_t strA;
  uint16_t distB;
  uint16_t strB;

  // ---- Validity flags ----
  bool validA;
  bool validB;

  //----fan state-----
  bool fanState;   // NEW FIELD

};

// Initialize CAN receiver (TWAI)
void CAN_RX_begin();

// Read from CAN bus, update data, and return both raw + filtered readings
LidarData CAN_RX_read();
void CAN_sendFanCommand(bool on);
#endif // CAN_RX_H