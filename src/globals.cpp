#include "globals.h"

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Bsec iaqSensor;

char oledBuffer[64];

AppMode currentAppMode = MODE_OFFLINE;
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
ThermalState thermal = THERM_NORMAL;

EnvironmentData envData = {
  0.0f, // temperature
  0.0f, // humidity
  0.0f, // pressure
  0.0f, // altitude
  0.0f, // gasResistance
  NAN,  // gasResistanceEMA
  NAN,  // iaq
  NAN,  // iaqStatic
  NAN,  // iaqStaticDisp
  0,    // iaqAccuracy
  0,    // iaqAccuracyPrev
  0     // iaqAccuracyDisp
};

float seaLevelPressure_hPa_current = 1012.50f;
float gasBaseline_kOhm = NAN;
bool gasBaselineReady = false;

unsigned long bootMs = 0;
unsigned long lastSensorReadMillis = 0;
unsigned long oledScreenStateChangeMillis = 0;
unsigned long nextSensorRetryMillis = 0;
unsigned long lastBsecSaveMs = 0;
unsigned long nextSafetyProcessMs = 0;
unsigned long sensorRetryBackoffMs = 1000;

bool bsecActive = false;
bool bsecHasData = false;
unsigned long lastBsecDataMs = 0;

float prev_T = NAN, prev_H = NAN, prev_P = NAN, prev_Alt = NAN;
float prev_G = NAN, prev_IAQ = NAN;
uint8_t prev_Acc = 255;
const char *prev_AQS = "";
unsigned long prevUptimeSec = 0;
OledDisplayState lastDrawnState = OLED_STATE_ERROR_SCREEN;

bool errorScreenDrawn = false;

uint8_t overheatPosIndex = 0;

float iaqMeanEWMA = NAN;
float iaqVarEWMA = 0.0f;

uint32_t gasSampleCounter = 0;

float portBufP[PORT_BUF] = {0};
float portBufH[PORT_BUF] = {0};
float portBufI[PORT_BUF] = {0};
uint8_t portFill = 0, portW = 0, portDecim = 0, portStableCount = 0;
bool baselineFrozen = false;

float gAltSmooth = NAN;
float altRaw3[3] = {NAN, NAN, NAN};
uint8_t altIdx = 0, altCnt = 0;

char wifiSsid[WIFI_SSID_LEN] = {0};
char wifiPass[WIFI_PASS_LEN] = {0};
float wifiLat = DEFAULT_LAT;
float wifiLon = DEFAULT_LON;
bool qnhAutoEnabled = true;
QnhSource qnhSource = QNH_SOURCE_DEFAULT;
unsigned long lastQnhSyncMs = 0;
int wifiLastHttpCode = 0;
int wifiLastStatus = 0;
uint8_t wifiFailCount = 0;
