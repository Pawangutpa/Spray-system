// Use this below code block to get data to main.ino file 
/* 

#include <Arduino.h>
#include "Proxi.h"

void setup() {
  Serial.begin(115200);
  Proxi_begin();
}

void loop() {
  ProxiData d = Proxi_read();

  Serial.print("Sensor 1: ");
  Serial.print(d.count1);
  Serial.print(" || Sensor 2: ");
  Serial.print(d.count2);
  Serial.print(" || Sensor 3: ");
  Serial.println(d.count3);

  delay(200); // same as your original
}

*/


#ifndef PROXI_H
#define PROXI_H

#include <Arduino.h>

struct ProxiData {
  unsigned long count1;
  unsigned long count2;
  unsigned long count3;
  float count_avg;   // average of count2 and count3
};

void Proxi_begin();
ProxiData Proxi_read();

#endif // PROXI_H