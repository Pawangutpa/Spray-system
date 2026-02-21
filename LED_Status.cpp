#include "LED_Status.h"

/* -------- GPIO -------- */
static uint8_t PIN_R, PIN_G, PIN_B;

/* -------- State -------- */
static LedState currentState = LED_BOOTING;
static LedColor currentColor = LED_OFF;

static uint32_t blinkPeriodMs = 0;
static uint32_t lastToggleMs = 0;
static bool ledOn = false;

/* -------- Common Anode Write -------- */
static void writeRGB(LedColor c, bool on)
{
    bool r = true, g = true, b = true;   // HIGH = OFF (common anode)

    if (on) {
        switch (c) {
            case LED_RED:     r = false; break;
            case LED_GREEN:   g = false; break;
            case LED_BLUE:    b = false; break;
            case LED_YELLOW:  r = g = false; break;
            case LED_CYAN:    g = b = false; break;
            case LED_MAGENTA: r = b = false; break;
            case LED_WHITE:   r = g = b = false; break;
            default: break;
        }
    }

    digitalWrite(PIN_R, r);
    digitalWrite(PIN_G, g);
    digitalWrite(PIN_B, b);
}

/* -------- Apply State -------- */
static void applyState(LedState s)
{
    switch (s) {

        case LED_BOOTING:
            currentColor = LED_WHITE;
            blinkPeriodMs = 200;
            break;

        case LED_SENSOR_ERROR:
            currentColor = LED_RED;
            blinkPeriodMs = 150;
            break;

        case LED_SIM_NOT_READY:
            currentColor = LED_RED;
            blinkPeriodMs = 1000;
            break;

        case LED_INTERNET_DOWN:
            currentColor = LED_YELLOW;
            blinkPeriodMs = 800;
            break;

        case LED_MQTT_DISCONNECTED:
            currentColor = LED_BLUE;
            blinkPeriodMs = 800;
            break;

        case LED_MQTT_PUBLISHING:
            currentColor = LED_CYAN;
            blinkPeriodMs = 120;
            break;

        case LED_GPS_NO_FIX:
            currentColor = LED_MAGENTA;
            blinkPeriodMs = 600;
            break;

        case LED_MANUAL_MODE:
            currentColor = LED_GREEN;
            blinkPeriodMs = 1000;
            break;

        case LED_AUTO_MODE:
            currentColor = LED_GREEN;
            blinkPeriodMs = 0;   // solid
            break;

        case LED_ALL_OK:
            currentColor = LED_WHITE;
            blinkPeriodMs = 0;
            break;
    }

    ledOn = true;
    lastToggleMs = millis();
    writeRGB(currentColor, true);
}

/* -------- Public -------- */
void LED_begin(uint8_t pinR, uint8_t pinG, uint8_t pinB)
{
    PIN_R = pinR;
    PIN_G = pinG;
    PIN_B = pinB;

    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_B, OUTPUT);

    writeRGB(LED_OFF, false);
}

void LED_setState(LedState s)
{
    if (s != currentState) {
        currentState = s;
        applyState(s);
    }
}

void LED_tick()
{
    if (blinkPeriodMs == 0) return;

    uint32_t now = millis();
    if (now - lastToggleMs >= blinkPeriodMs) {
        lastToggleMs = now;
        ledOn = !ledOn;
        writeRGB(currentColor, ledOn);
    }
}
