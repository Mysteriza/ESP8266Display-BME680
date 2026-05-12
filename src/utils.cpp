#include "utils.h"

const char *getIaqCategory(float x)
{
  if (isnan(x))
    return "n/a";
  if (x <= 50.0f)
    return "Excellent";
  if (x <= 100.0f)
    return "Good";
  if (x <= 150.0f)
    return "Light";
  if (x <= 200.0f)
    return "Moderate";
  if (x <= 300.0f)
    return "Unhealthy";
  return "Hazardous";
}

float med3(float a, float b, float c)
{
  if (a > b)
  {
    float t = a;
    a = b;
    b = t;
  }
  if (b > c)
  {
    float t = b;
    b = c;
    c = t;
  }
  if (a > b)
  {
    float t = a;
    a = b;
    b = t;
  }
  return b;
}

float simpleBaroAltitude(float press_hPa, float qnh_hPa)
{
  if (!(press_hPa > 0 && qnh_hPa > 0))
    return NAN;
  return 44330.0f * (1.0f - powf(press_hPa / qnh_hPa, 0.190294957f));
}

float qnhFromRef(float press_hPa, float href_m)
{
  float k = 1.0f - (href_m / 44330.0f);
  if (k <= 0.0f)
    return NAN;
  return press_hPa / powf(k, 5.255f);
}

bool hasChanged(float current, float previous, float threshold)
{
  if (isnan(current) || isnan(previous))
    return true;
  return fabsf(current - previous) >= threshold;
}
