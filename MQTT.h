#ifndef MQTT_H
#define MQTT_H

#include <Arduino.h>
#include "CAN_RX.h"
#include "Proxi.h"
#include "button.h"



// ===== MQTT STATUS (LED SUPPORT) =====
bool MQTT_isSimRegistered();
bool MQTT_isInternetUp();
bool MQTT_isConnected();

// init
void MQTT_begin(uint32_t baud = 115200);

// non-blocking queue APIs (return false if busy)
bool MQTT_publishHeartbeat(const LidarData &lidar, const ProxiData &prox, const ButtonData &btn);

// MQTT.h additions — tiny getters for main.ino
double MQTT_getLatitude();
double MQTT_getLongitude();
int    MQTT_getSatellites();
int    MQTT_getFixQuality();
//bool MQTT_publishDashboard(const LidarData &lidar, const ProxiData &prox);

// must be called frequently from main loop
void MQTT_tick();

#endif // MQTT_H