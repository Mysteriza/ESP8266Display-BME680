#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

bool initBSEC();
void setQNH(float qnh);
void calQNHFromAltRef(float href_m);
void bsecLoopTick();
void readBME680SensorData();
void handleSensorAutoRetry();
void scheduleSensorRetryInitial();
void enterHotHold();
void exitHotHold();
void resetAltitudeFiltering();

#endif
