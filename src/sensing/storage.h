#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#include <EEPROM.h>

bool loadBsecState();
void saveBsecState();
void loadPersistent();
void saveSeaLevelPressure(float p);
void saveGasBaseline(float b, bool rdy);

#endif
