#include "globals.h"

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Bsec iaqSensor;

char oledBuffer[64];

AppMode currentAppMode = MODE_OFFLINE;
PowerState currentPowerState = POWER_ACTIVE;
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
ThermalState thermal = THERM_NORMAL;

float gTemp = 0.0f;
float gHum = 0.0f;
float gPress = 0.0f;
float gAlt = 0.0f;
float gGas_kOhm = 0.0f;
float gGasEMA_kOhm = NAN;
float gIAQ = NAN;
float gIAQstatic = NAN;
float gIAQstaticDisp = NAN;
uint8_t gIAQacc = 0;
uint8_t gIAQaccPrev = 0;
uint8_t gIAQaccDisp = 0;

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
