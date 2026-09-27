#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <math.h>

const char *getIaqCategory(float x);

float med3(float a, float b, float c);

float temperatureCompensatedAltitude(float press_hPa, float qnh_hPa, float temp_C);

float qnhFromRef(float press_hPa, float href_m);

bool hasChanged(float current, float previous, float threshold);

#endif
