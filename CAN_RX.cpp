#include "CAN_RX.h"
#include "driver/twai.h"

const gpio_num_t CAN_TX = (gpio_num_t)36;
const gpio_num_t CAN_RX = (gpio_num_t)37;

// ----------------- RAW storage -----------------
static uint16_t rawDistA = 0, rawStrA = 0;
static uint16_t rawDistB = 0, rawStrB = 0;

// NEW: Store latest fan state received
static bool lastFanState = false;

// ----------------- Filter parameters -----------------
static const uint16_t MIN_STRENGTH = 1000;
static const uint16_t MIN_DISTANCE_CM = 30;
static const uint16_t MAX_DISTANCE_CM = 200;

// Median + N-of-M window sizes
static const int MEDIAN_WINDOW = 3;
static const int VALID_M = 5;
static const int VALID_N = 3;

// ----------------- Filter state -----------------
static uint16_t medBufA[MEDIAN_WINDOW] = {0};
static uint16_t medBufB[MEDIAN_WINDOW] = {0};
static int medIdxA = 0, medIdxB = 0;

static bool validWinA[VALID_M] = {false};
static bool validWinB[VALID_M] = {false};
static int winIdxA = 0, winIdxB = 0;

// ---- median of 3 ----
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// ---- filter functions ----
static void processFilters(uint16_t latestDist, uint16_t latestStr,
                           uint16_t &outDist, uint16_t &outStr,
                           bool &outValid,
                           uint16_t *medBuf, int &medIdx,
                           bool *validWin, int &winIdx) {

  // update median buffer
  medBuf[medIdx] = latestDist;
  medIdx = (medIdx + 1) % MEDIAN_WINDOW;
  uint16_t med = median3(medBuf[0], medBuf[1], medBuf[2]);

  // per-frame gates
  bool frame_ok = !(latestStr < MIN_STRENGTH ||
                    latestDist < MIN_DISTANCE_CM ||
                    latestDist > MAX_DISTANCE_CM);

  // update validity window
  validWin[winIdx] = frame_ok;
  winIdx = (winIdx + 1) % VALID_M;

  // count trues
  int cnt = 0;
  for (int i = 0; i < VALID_M; ++i) if (validWin[i]) ++cnt;
  bool valid = (cnt >= VALID_N);

  if (valid) {
    outDist = med;
    outStr = latestStr;
    outValid = true;
  } else {
    outDist = 0;
    outStr = 0;
    outValid = false;
  }
}

// ---- TWAI setup ----
void CAN_RX_begin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g, &t, &f) == ESP_OK) Serial.println("CAN RX: Driver installed");
  if (twai_start() == ESP_OK) Serial.println("CAN RX: Started");
}



void CAN_sendFanCommand(bool on) {
  twai_message_t msg{};
  msg.identifier = 0x200;         // Fan command CAN ID
  msg.data_length_code = 1;       // Only send 1 byte
  msg.data[0] = on ? 1 : 0;       // 1 = fan ON, 0 = fan OFF

  twai_transmit(&msg, pdMS_TO_TICKS(5));
}


// ---- main read ----
LidarData CAN_RX_read() {

  twai_message_t rx{};

  // Accept both 4-byte (old) and 5-byte (new: fan state)
  if (twai_receive(&rx, pdMS_TO_TICKS(200)) == ESP_OK && rx.data_length_code >= 4) {

    uint16_t distance = rx.data[0] | (rx.data[1] << 8);
    uint16_t strength = rx.data[2] | (rx.data[3] << 8);

    // -------- Sensor A --------
    if (rx.identifier == 0x101) {
      rawDistA = distance;
      rawStrA  = strength;

      if (rx.data_length_code >= 5) {
        lastFanState = rx.data[4];
      }
    }

    // -------- Sensor B --------
    else if (rx.identifier == 0x102) {
      rawDistB = distance;
      rawStrB  = strength;

      if (rx.data_length_code >= 5) {
        lastFanState = rx.data[4];
      }
    }
  }

  // Apply filtering
  uint16_t filtDistA, filtStrA, filtDistB, filtStrB;
  bool validA, validB;

  processFilters(rawDistA, rawStrA, filtDistA, filtStrA, validA,
                 medBufA, medIdxA, validWinA, winIdxA);

  processFilters(rawDistB, rawStrB, filtDistB, filtStrB, validB,
                 medBufB, medIdxB, validWinB, winIdxB);

  // Pack final struct
  LidarData data = {
    rawDistA, rawStrA, rawDistB, rawStrB,
    filtDistA, filtStrA, filtDistB, filtStrB,
    validA, validB,
    lastFanState        // <-- NEW FIELD RETURNED
  };

  return data;
}