#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <Arduino.h>

/* ================= COLORS ================= */
enum LedColor {
    LED_OFF,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_YELLOW,
    LED_CYAN,
    LED_MAGENTA,
    LED_WHITE
};

/* ================= STATES ================= */
enum LedState {
    LED_BOOTING,

    LED_SENSOR_ERROR,

    LED_SIM_NOT_READY,
    LED_INTERNET_DOWN,
    LED_MQTT_DISCONNECTED,

    LED_MQTT_PUBLISHING,

    LED_GPS_NO_FIX,

    LED_MANUAL_MODE,
    LED_AUTO_MODE,

    LED_ALL_OK
};

/* ================= API ================= */
void LED_begin(uint8_t pinR, uint8_t pinG, uint8_t pinB);
void LED_setState(LedState s);
void LED_tick();   // call in loop()

#endif
