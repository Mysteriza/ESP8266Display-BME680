#ifndef TYPES_H
#define TYPES_H

#include "config.h"

enum AppMode
{
  MODE_OFFLINE,
  MODE_BME_ERROR
};

enum OledDisplayState
{
  OLED_STATE_ERROR_SCREEN,
  OLED_STATE_DATA_SCREEN_1,
  OLED_STATE_DATA_SCREEN_2,
  OLED_STATE_DATA_SCREEN_3,
  OLED_STATE_DATA_SCREEN_4,
  OLED_STATE_OFF
};

enum ThermalState
{
  THERM_NORMAL,
  THERM_HOT_HOLD
};

/// @brief One stored WiFi network (fixed-size, EEPROM-friendly)
struct WifiCred
{
  char ssid[WIFI_SSID_LEN];
  char pass[WIFI_PASS_LEN];
};

/// @brief Origin of the active QNH value (drives altitude)
enum QnhSource : uint8_t
{
  QNH_SOURCE_DEFAULT, ///< EEPROM default / fallback
  QNH_SOURCE_MANUAL,  ///< Serial QNH= or ALTREF= (disables auto)
  QNH_SOURCE_AUTO     ///< Open-Meteo periodic sync
};

#endif
