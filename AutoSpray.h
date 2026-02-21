#ifndef AUTOSPRAY_H
#define AUTOSPRAY_H

#include <Arduino.h>
#include "Proxi.h"

// Side enum used by AutoSpray functions (matches your .cpp)
typedef enum { SIDE_LEFT = 0, SIDE_RIGHT = 1 } Side;

// lifecycle / configuration
void AutoSpray_begin();
void AutoSpray_setPins(int leftPin, int rightPin);

// feed raw lidar frame values (distance in cm, strength)
void AutoSpray_feedRaw(Side side, uint16_t rawDist_cm, uint16_t rawStr);

// run frequently to actuate solenoids from queued segments
void AutoSpray_process();

// debug flags (set by AutoSpray.cpp when segments are enqueued)
extern volatile bool auto_enqueued_left;
extern volatile bool auto_enqueued_right;

#endif // AUTOSPRAY_H