#include "button.h"

// Pin mapping
const int sw_left       = 1;
const int sw_autonomous = 40;
const int sw_right      = 41;
const int sw_both       = 42;

void Button_begin() {
  pinMode(sw_left , INPUT_PULLUP);
  pinMode(sw_autonomous, INPUT_PULLUP);
  pinMode(sw_right, INPUT_PULLUP);
  pinMode(sw_both, INPUT_PULLUP);
}

ButtonData Button_read() {
  ButtonData b;

  // LOW means pressed (because of INPUT_PULLUP)
  b.left       = (digitalRead(sw_left) == LOW);
  b.autonomous = (digitalRead(sw_autonomous) == LOW);
  b.right      = (digitalRead(sw_right) == LOW);
  b.both       = (digitalRead(sw_both) == LOW);

  return b;
}
