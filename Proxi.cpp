#include "Proxi.h"

/* ============================
   PIN DEFINITIONS
   ============================ */

// Flow sensor (water meter)
const int intPin1 = 3;   // Flow sensor pulse output

// Nut counter sensors
const int intPin2 = 6;   // Nut counter sensor 1
const int intPin3 = 7;   // Nut counter sensor 2

/* ============================
   FLOW SENSOR CONFIG
   ============================ */

// Calibration constant:
// 274 pulses = 1 liter of water
#define FLOW_PULSES_PER_LITER 274.0

/* ============================
   VOLATILE VARIABLES (ISR)
   ============================ */

// Total pulse count from flow sensor
volatile unsigned long flowPulseCount = 0;

// Nut counters
volatile unsigned long nutCount2 = 0;
volatile unsigned long nutCount3 = 0;

// Debounce timing for nut sensors (milliseconds)
volatile unsigned long lastInterruptTime2 = 0;
volatile unsigned long lastInterruptTime3 = 0;

/* ============================
   ISR DECLARATIONS
   ============================ */

void IRAM_ATTR flowPulseISR();   // Flow sensor interrupt
void IRAM_ATTR countNuts2();     // Nut sensor 2 interrupt
void IRAM_ATTR countNuts3();     // Nut sensor 3 interrupt

/* ============================
   INITIALIZATION
   ============================ */

void Proxi_begin() {

  // Configure sensor pins as input with pull-up resistors
  // (Typical for open-collector proximity / flow sensors)
  pinMode(intPin1, INPUT_PULLUP);
  pinMode(intPin2, INPUT_PULLUP);
  pinMode(intPin3, INPUT_PULLUP);

  // Attach interrupts on rising edge (LOW → HIGH)
  attachInterrupt(digitalPinToInterrupt(intPin1), flowPulseISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin2), countNuts2,  RISING);
  attachInterrupt(digitalPinToInterrupt(intPin3), countNuts3,  RISING);
}

/* ============================
   MAIN READ FUNCTION
   ============================ */

ProxiData Proxi_read() {
  ProxiData r;

  unsigned long pulses;
  unsigned long c2, c3;

  // --- Safely copy ISR-updated values ---
  noInterrupts();
  pulses = flowPulseCount;
  c2 = nutCount2;
  c3 = nutCount3;
  interrupts();

  /* ============================
     WATER METER (ACCUMULATED)
     ============================ */

  // Retains value across function calls
  static unsigned long lastPulseSnapshot = 0;
  static float totalWaterLiters = 0.0;

  // Update total water only if new pulses arrived
//   if (pulses != lastPulseSnapshot) {
//     unsigned long delta = pulses - lastPulseSnapshot;
//     totalWaterLiters += delta / FLOW_PULSES_PER_LITER;
//     lastPulseSnapshot = pulses;
// }

if (pulses != lastPulseSnapshot) {
    lastPulseSnapshot = pulses;
    totalWaterLiters += lastPulseSnapshot/ FLOW_PULSES_PER_LITER;
    
}


  // Always report total accumulated water
  r.count1 = totalWaterLiters;

  /* ============================
     NUT COUNTERS
     ============================ */

  r.count2 = c2;
  r.count3 = c3;
  r.count_avg = (r.count2 + r.count3) / 2.0;

  return r;
}

/* ============================
   INTERRUPT SERVICE ROUTINES
   ============================ */

// Flow sensor ISR
// Increments pulse count on every water flow pulse
void IRAM_ATTR flowPulseISR() {
   static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last > 50000) {  // 50ms Debounce
    flowPulseCount++;;
    last = now;
  }
}

// Nut sensor 2 ISR with debounce protection
void IRAM_ATTR countNuts2() {
  unsigned long t = millis();

  if (t - lastInterruptTime2 > 10) {
    nutCount2++;
    lastInterruptTime2 = t;
  }
}

// Nut sensor 3 ISR with debounce protection
void IRAM_ATTR countNuts3() {
  unsigned long t = millis();

  if (t - lastInterruptTime3 > 10) {
    nutCount3++;
    lastInterruptTime3 = t;
  }
}