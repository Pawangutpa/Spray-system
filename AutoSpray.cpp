#include "AutoSpray.h"
#include "CAN_RX.h"
#include "Proxi.h"
#include <limits.h>

static const uint16_t MIN_DIST_CM = 20;
static const uint16_t MAX_DIST_CM = 400;
static const uint16_t MIN_STR = 200;
static const int OFFSET_COUNT = 6;  // offset between LiDAR and nozzle (~2.57 m)

static const int QUEUE_CAP = 24;
static const uint32_t END_OPEN = UINT32_MAX; // marker for open-ended segment

struct SegQueue {
  uint32_t s[QUEUE_CAP];
  uint32_t e[QUEUE_CAP];
  uint8_t head = 0, tail = 0, sz = 0;
  void push(uint32_t a, uint32_t b) {
    if (sz >= QUEUE_CAP) { head = (head + 1) % QUEUE_CAP; sz--; }
    s[tail] = a; e[tail] = b; tail = (tail + 1) % QUEUE_CAP; sz++;
  }
  bool empty() { return sz == 0; }
  void pop() { if (sz == 0) return; head = (head + 1) % QUEUE_CAP; sz--; }
  uint32_t frontS() { return s[head]; }
  uint32_t frontE() { return e[head]; }
  void clear() { head = tail = sz = 0; }
  // set end for the most recent (tail-1) entry (used to fill provisional segment)
  void setLastEnd(uint32_t endVal) {
    if (sz == 0) return;
    int idx = (tail + QUEUE_CAP - 1) % QUEUE_CAP;
    e[idx] = endVal;
  }
  // check if last segment is open (end == END_OPEN)
  bool lastIsOpen() {
    if (sz == 0) return false;
    int idx = (tail + QUEUE_CAP - 1) % QUEUE_CAP;
    return e[idx] == END_OPEN;
  }
} qL, qR;

// canopy state
static bool inCanopyL = false, inCanopyR = false;
static uint32_t startCntL = 0, startCntR = 0;

// solenoid pins & state
static int Left_relay = 4, Right_relay = 5;
static bool solL = false, solR = false;

// debug flags (extern)
volatile bool auto_enqueued_left = false;
volatile bool auto_enqueued_right = false;

static inline uint32_t getCount() {
  ProxiData p = Proxi_read();
  if (p.count_avg > 0.0f) return (uint32_t)round(p.count_avg);
  return p.count1;
}
 
static inline void setSolL(bool on) { if (Left_relay >= 0) digitalWrite(Left_relay, on ? HIGH : LOW); solL = on; }
static inline void setSolR(bool on) { if (Right_relay >= 0) digitalWrite(Right_relay, on ? HIGH : LOW); solR = on; }

void AutoSpray_begin() {
  pinMode(Left_relay, OUTPUT);
  pinMode(Right_relay, OUTPUT);
  setSolL(false);
  setSolR(false);
  qL.clear();
  qR.clear();
  inCanopyL = inCanopyR = false;
  startCntL = startCntR = 0;
  auto_enqueued_left = auto_enqueued_right = false;
}

void AutoSpray_setPins(int leftPin, int rightPin) {
  Left_relay = leftPin;
  Right_relay = rightPin;
  pinMode(Left_relay, OUTPUT);
  pinMode(Right_relay, OUTPUT);
  setSolL(false);
  setSolR(false);
}

// ---------- CHANGED: enqueue start immediately (end=END_OPEN), fill end at canopy end ----------
void AutoSpray_feedRaw(Side side, uint16_t rawDist_cm, uint16_t rawStr) {
  uint32_t now = getCount();
  bool isCanopy = (rawStr > MIN_STR) && (rawDist_cm >= MIN_DIST_CM) && (rawDist_cm <= MAX_DIST_CM);

  // If valid canopy reading
  if (isCanopy) {
    if (side == SIDE_LEFT) {
      if (!inCanopyL) {
        // canopy START: record start and push provisional segment with END_OPEN
        inCanopyL = true;
        startCntL = now;
        uint32_t qStart = startCntL + OFFSET_COUNT;
        qL.push(qStart, END_OPEN); // end will be filled when canopy ends
        auto_enqueued_left = true;
        Serial.print("[ENQ L:start] startCnt="); Serial.print(startCntL);
        Serial.print(" qStart="); Serial.println(qStart);
      }
    } else { // right
      if (!inCanopyR) {
        inCanopyR = true;
        startCntR = now;
        uint32_t qStart = startCntR + OFFSET_COUNT;
        qR.push(qStart, END_OPEN);
        auto_enqueued_right = true;
        Serial.print("[ENQ R:start] startCnt="); Serial.print(startCntR);
        Serial.print(" qStart="); Serial.println(qStart);
      }
    }
    return;
  }

  // Not canopy: if we were inside canopy, mark its end now and fill last queued segment's end
  if (side == SIDE_LEFT && inCanopyL) {
    inCanopyL = false;
    uint32_t qEnd = now + OFFSET_COUNT;
    // Ensure there is an open segment to fill
    if (qL.lastIsOpen()) {
      qL.setLastEnd(qEnd);
      Serial.print("[ENQ L:end] now="); Serial.print(now);
      Serial.print(" qEnd="); Serial.println(qEnd);
    } else {
      // no open found — push a closed segment (fallback)
      qL.push(startCntL + OFFSET_COUNT, qEnd);
      Serial.println("[ENQ L:end-fallback]");
    }
  }

  if (side == SIDE_RIGHT && inCanopyR) {
    inCanopyR = false;
    uint32_t qEnd = now + OFFSET_COUNT;
    if (qR.lastIsOpen()) {
      qR.setLastEnd(qEnd);
      Serial.print("[ENQ R:end] now="); Serial.print(now);
      Serial.print(" qEnd="); Serial.println(qEnd);
    } else {
      qR.push(startCntR + OFFSET_COUNT, qEnd);
      Serial.println("[ENQ R:end-fallback]");
    }
  }
}

// process queue and actuate
void AutoSpray_process() {
  uint32_t cur = getCount();

  // LEFT
  if (!qL.empty()) {
    uint32_t s = qL.frontS();
    uint32_t e = qL.frontE();
    // if the queued segment is still open (end==END_OPEN), we won't turn off until it's filled
    if (!solL && cur >= s) {
      Serial.print("[ACTION L] ON cur="); Serial.print(cur); Serial.print(" s="); Serial.println(s);
      setSolL(true);
    }
    if (solL && e != END_OPEN && cur >= e) {
      Serial.print("[ACTION L] OFF cur="); Serial.print(cur); Serial.print(" e="); Serial.println(e);
      setSolL(false);
      qL.pop();
    }
  }

  // RIGHT
  if (!qR.empty()) {
    uint32_t s = qR.frontS();
    uint32_t e = qR.frontE();
    if (!solR && cur >= s) {
      Serial.print("[ACTION R] ON cur="); Serial.print(cur); Serial.print(" s="); Serial.println(s);
      setSolR(true);
    }
    if (solR && e != END_OPEN && cur >= e) {
      Serial.print("[ACTION R] OFF cur="); Serial.print(cur); Serial.print(" e="); Serial.println(e);
      setSolR(false);
      qR.pop();
    }
  }
}