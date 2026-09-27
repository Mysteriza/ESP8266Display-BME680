#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

const char *getIaqCategory(float x);

float med3(float a, float b, float c);

float temperatureCompensatedAltitude(float press_hPa, float qnh_hPa, float temp_C);

float qnhFromRef(float press_hPa, float href_m);

bool hasChanged(float current, float previous, float threshold);

// Mean hourly rate over stored QNH history: (newest-oldest)/(n-1).
// Returns 0 when fewer than 2 samples exist.
float qnhTrendPerHour(const float *hist, uint8_t n);

// Display arrow for a trend rate: '^' rising, 'v' falling, '~' steady.
char qnhTrendArrow(float ratePerHour);

#endif
