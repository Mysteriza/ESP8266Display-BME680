#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"
#include "types.h"

extern Adafruit_SSD1306 display;
extern Bsec iaqSensor;

extern char oledBuffer[64];

extern AppMode currentAppMode;
extern OledDisplayState currentOledScreenState;
extern ThermalState thermal;

struct EnvironmentData {
  float temperature;
  float humidity;
  float pressure;
  float altitude;
  float gasResistance;
  float gasResistanceEMA;
  float iaq;
  float iaqStatic;
  float iaqStaticDisp;
  uint8_t iaqAccuracy;
  uint8_t iaqAccuracyPrev;
  uint8_t iaqAccuracyDisp;
};

extern EnvironmentData envData;

extern float seaLevelPressure_hPa_current;
extern float gasBaseline_kOhm;
extern bool gasBaselineReady;

extern unsigned long bootMs;
extern unsigned long lastSensorReadMillis;
extern unsigned long oledScreenStateChangeMillis;
extern unsigned long nextSensorRetryMillis;
extern unsigned long lastBsecSaveMs;
extern unsigned long nextSafetyProcessMs;
extern unsigned long sensorRetryBackoffMs;

extern bool bsecActive;
extern bool bsecHasData;
extern unsigned long lastBsecDataMs;

extern float prev_T, prev_H, prev_P, prev_Alt;
extern float prev_G, prev_IAQ;
extern uint8_t prev_Acc;
extern const char *prev_AQS;
extern unsigned long prevUptimeSec;
extern OledDisplayState lastDrawnState;

extern bool errorScreenDrawn;

extern uint8_t overheatPosIndex;

extern float iaqMeanEWMA;
extern float iaqVarEWMA;

extern uint32_t gasSampleCounter;

extern float portBufP[PORT_BUF];
extern float portBufH[PORT_BUF];
extern float portBufI[PORT_BUF];
extern uint8_t portFill, portW, portDecim, portStableCount;
extern bool baselineFrozen;

extern float gAltSmooth;
extern float altRaw3[3];
extern uint8_t altIdx, altCnt;

// WiFi periodic sync + auto-QNH state
extern char wifiSsid[WIFI_SSID_LEN];
extern char wifiPass[WIFI_PASS_LEN];
extern float wifiLat;
extern float wifiLon;
extern bool qnhAutoEnabled;
extern QnhSource qnhSource;
extern unsigned long lastQnhSyncMs;
extern int wifiLastHttpCode;
extern int wifiLastStatus;
extern uint8_t wifiFailCount;

#endif
