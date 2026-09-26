#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

bool initBSEC();
bool setQNH(float qnh);
bool applyAutoQnh(float qnh);
void calQNHFromAltRef(float href_m);
void bsecLoopTick();
void readBME680SensorData();
void handleSensorAutoRetry();
void enterHotHold();
void exitHotHold();
void resetAltitudeFiltering();

#endif
