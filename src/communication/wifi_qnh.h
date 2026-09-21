#ifndef WIFI_QNH_H
#define WIFI_QNH_H

#include <Arduino.h>

// Periodic WiFi QNH sync (Option B): radio stays OFF, wakes hourly,
// fetches current pressure_msl from Open-Meteo, then sleeps again.
void wifiQnhBegin();
void wifiQnhTick();
bool wifiQnhHasCreds();
void wifiQnhForceSync();

#endif
