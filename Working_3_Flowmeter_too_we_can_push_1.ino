/// Everything is working . just we have to add led state machine from here

#include <Arduino.h>
#include "CAN_RX.h"
#include "Proxi.h"
#include "AutoSpray.h"
#include "MQTT.h"
#include "button.h"


//  LED state machine
#include "LED_Status.h"

#define SOL_LEFT_PIN   4
#define SOL_RIGHT_PIN  5

//  RGB LED pins (COMMON ANODE)
#define LED_R_PIN  8
#define LED_G_PIN  9
#define LED_B_PIN  10

// flags coming from AutoSpray.cpp
extern volatile bool auto_enqueued_left;
extern volatile bool auto_enqueued_right;

unsigned long lastPrint = 0;

void flowmeterTask(void *p);

/* =========================================================
   LED STATE DECISION LOGIC
   ========================================================= */
void updateLedState(const LidarData &lidar,
                    
                    const ButtonData &btn)
{
  // 1️⃣ SENSOR ERROR (highest priority)
  if (!lidar.validA || !lidar.validB ) {
    LED_setState(LED_SENSOR_ERROR);
    return;
  }

  // 2️⃣ SIM NOT READY
  if (!MQTT_isSimRegistered()) {
    LED_setState(LED_SIM_NOT_READY);
    return;
  }

  // 3️⃣ INTERNET / PDP CONTEXT DOWN
  if (!MQTT_isInternetUp()) {
    LED_setState(LED_INTERNET_DOWN);
    return;
  }

  // 4️⃣ MQTT DISCONNECTED
  if (!MQTT_isConnected()) {
    LED_setState(LED_MQTT_DISCONNECTED);
    return;
  }

  // 5️⃣ GPS NO FIX
  if (MQTT_getFixQuality() < 3) {
    LED_setState(LED_GPS_NO_FIX);
    return;
  }

  // 6️⃣ MODE (lowest priority)
  if (btn.autonomous) {
    LED_setState(LED_AUTO_MODE);
  } else {
    LED_setState(LED_MANUAL_MODE);
  }
}

/* =========================================================
   SETUP
   ========================================================= */
void setup() {
  Serial.begin(115200);

  // 🔵 LED INIT (booting indication)
  LED_begin(LED_R_PIN, LED_G_PIN, LED_B_PIN);
  LED_setState(LED_BOOTING);

 

  CAN_RX_begin();
  Proxi_begin();

  AutoSpray_setPins(SOL_LEFT_PIN, SOL_RIGHT_PIN);
  AutoSpray_begin();

  MQTT_begin();      // modem + network + mqtt
  Button_begin();
}

/* =========================================================
   LOOP
   ========================================================= */
void loop() {

  // ---- Read all inputs ----
  LidarData  lidar = CAN_RX_read();
  ProxiData  prox  = Proxi_read();
  ButtonData btn   = Button_read();

  // ---- Feed AutoSpray ----
  AutoSpray_feedRaw(SIDE_LEFT,  lidar.rawDistA, lidar.rawStrA);
  AutoSpray_feedRaw(SIDE_RIGHT, lidar.rawDistB, lidar.rawStrB);

  // ---- Manual / Auto logic ----
  if (btn.autonomous) {
    CAN_sendFanCommand(true);
    AutoSpray_process();
  } else {
    CAN_sendFanCommand(false);

    if (btn.left) {
      digitalWrite(SOL_LEFT_PIN, HIGH);
    } else if (btn.right) {
      digitalWrite(SOL_RIGHT_PIN, HIGH);
    } else if (btn.both) {
      digitalWrite(SOL_LEFT_PIN, HIGH);
      digitalWrite(SOL_RIGHT_PIN, HIGH);
    } else {
      digitalWrite(SOL_LEFT_PIN, LOW);
      digitalWrite(SOL_RIGHT_PIN, LOW);
    }
  }

  // ---- MQTT engine ----
  MQTT_tick();

  // ---- Periodic publish ----
  static unsigned long lastPubMs = 0;
  if (millis() - lastPubMs >= 2000) {
    MQTT_publishHeartbeat(lidar, prox, btn);
    lastPubMs = millis();
  }

  // 🔵 LED STATE UPDATE (non-blocking)
  updateLedState(lidar,btn);
  LED_tick();

  delay(5);  // keeps CPU stable, does NOT affect LED timing
}
