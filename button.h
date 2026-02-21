
#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

struct ButtonData {
  bool left;
  bool autonomous;
  bool right;
  bool both;
};

void Button_begin();
ButtonData Button_read();

#endif // BUTTON_H
