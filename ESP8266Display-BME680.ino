// ============================================================================
// SECTION 1: INCLUDES
// ============================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <bsec.h>
#include <math.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>

// ============================================================================
// SECTION 2: HARDWARE CONFIGURATION
// ============================================================================

// I2C Configuration
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define I2C_CLOCK_SLOW 100000UL // 100kHz for stability
#define I2C_CLOCK_FAST 400000UL // 400kHz for normal operation

// Display Configuration
#define OLED_ADDRESS 0x3C
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_RESET -1

// Sensor Configuration
#define BME_ADDRESS 0x76
#define BME_HEALING_MS 5000 // Initial BME680 heating period

// ============================================================================
// SECTION 3: APPLICATION CONFIGURATION
// ============================================================================

// Timing Configuration (continuous display mode - always on)
#define SENSOR_READ_INTERVAL_MS 30000UL    // Read sensor every 30s
#define OLED_DATA_SCREEN_1_DURATION 5000UL // Temp/Humidity screen duration
#define OLED_DATA_SCREEN_2_DURATION 5000UL // Pressure/Altitude screen duration
#define OLED_DATA_SCREEN_3_DURATION 5000UL // IAQ/Gas screen duration
#define OLED_DATA_SCREEN_4_DURATION 5000UL // Uptime screen duration

// Deep Sleep Configuration (power optimization)
#define DEEP_SLEEP_DURATION_US 30000000UL // 30 seconds between readings
#define ENABLE_DEEP_SLEEP true            // Set false to disable deep sleep

// Power Management Thresholds
#define BATTERY_CRITICAL_MV 3200 // 3.2V - enter critical mode
#define BATTERY_LOW_MV 3500      // 3.5V - reduce update rate
#define BATTERY_NORMAL_MV 3700   // 3.7V - normal operation

// Thermal Protection
#define HOT_ENTER_C 45.0f        // Enter overheat protection
#define HOT_EXIT_C 41.0f         // Exit overheat protection
#define SAFETY_PERIOD_MS 60000UL // Overheat display refresh rate

// BSEC Configuration
#define BSEC_SAVE_INTERVAL_MS 14400000UL    // Save BSEC state every 4 hours
#define BSEC_MIN_SAVE_GAP_MS 600000UL       // Minimum 10min between saves
#define BSEC_BOOT_STABILIZATION_MS 300000UL // 5min stabilization period

// ============================================================================
// SECTION 4: ALGORITHM PARAMETERS
// ============================================================================

// IAQ Filtering
#define IAQ_VAR_ALPHA 0.10f        // EWMA smoothing factor
#define IAQ_DISPLAY_MIN_ACCURACY 2 // Minimum accuracy to display

// Gas Resistance Baseline
#define GAS_EMA_ALPHA 0.20f             // EMA smoothing for gas
#define BASELINE_ALPHA_UP 0.02f         // Baseline adaptation (increase)
#define BASELINE_ALPHA_DOWN 0.001f      // Baseline adaptation (decrease)
#define GAS_BASELINE_READY_SAMPLES 2880 // ~24 hours at 30s intervals

// Transport Detection (baseline freeze during movement)
#define PORT_DECIM_N 10    // Decimation factor
#define PORT_BUF 24        // History buffer size
#define PORT_TH_DP 3.5f    // Pressure change threshold
#define PORT_TH_DH 8.0f    // Humidity change threshold
#define PORT_TH_DIAQ 25.0f // IAQ change threshold
#define PORT_RESUME_OK 6   // Stable periods to resume

// Altitude Filtering
#define ALT_EMA_ALPHA 0.12f  // EMA smoothing for altitude
#define ALT_OUTLIER_M 18.0f  // Outlier rejection threshold
#define ALT_DEADBAND_M 0.40f // Deadband for smooth updates

// Display Update Thresholds (reduce flicker)
#define TH_T 0.10f   // Temperature change threshold
#define TH_H 0.10f   // Humidity change threshold
#define TH_P 0.10f   // Pressure change threshold
#define TH_ALT 1.0f  // Altitude change threshold
#define TH_G 0.10f   // Gas resistance threshold
#define TH_IAQ 0.50f // IAQ change threshold

// OLED Contrast
#define CONTRAST_NORMAL 0x7F   // Normal contrast (50%)
#define CONTRAST_OVERHEAT 0x04 // Minimal contrast (overheat)

// ============================================================================
// SECTION 5: EEPROM MEMORY MAP
// ============================================================================

#define EEPROM_SIZE 512
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MAGIC_VALUE 0xB6680A11

// Stored configuration
#define SEA_LEVEL_PRESSURE_ADDR 4
#define GAS_BASELINE_ADDR 8
#define GAS_BASELINE_READY_ADDR 12

// BSEC state blob
#define BSEC_STATE_ADDR 16
#define BSEC_STATE_MAXLEN BSEC_MAX_STATE_BLOB_SIZE
#define BSEC_STATE_VALID_ADDR (BSEC_STATE_ADDR + BSEC_STATE_MAXLEN)
#define BSEC_STATE_VALID_MAGIC 0xB5EC1A0F

// ============================================================================
// SECTION 6: TYPE DEFINITIONS & ENUMERATIONS
// ============================================================================

/// @brief Application operating modes
enum AppMode
{
  MODE_OFFLINE,       ///< Normal offline operation
  MODE_BME_ERROR,     ///< BME680 sensor error
  MODE_POWER_CRITICAL ///< Low battery - minimal operation
};

/// @brief OLED display states
enum OledDisplayState
{
  OLED_STATE_ERROR_SCREEN,  ///< Error display
  OLED_STATE_DATA_SCREEN_1, ///< Temperature & Humidity
  OLED_STATE_DATA_SCREEN_2, ///< Pressure & Altitude
  OLED_STATE_DATA_SCREEN_3, ///< Gas & IAQ
  OLED_STATE_DATA_SCREEN_4, ///< Uptime
  OLED_STATE_OFF            ///< Display off (power saving)
};

/// @brief Thermal protection states
enum ThermalState
{
  THERM_NORMAL,  ///< Normal operation
  THERM_HOT_HOLD ///< Overheat protection active
};

/// @brief Power states for state machine
enum PowerState
{
  POWER_ACTIVE,      ///< Fully operational
  POWER_DISPLAY_OFF, ///< Display off, sensors active
  POWER_SLEEP_PREP,  ///< Preparing for sleep
  POWER_CRITICAL     ///< Critical battery mode
};

// ============================================================================
// SECTION 7: GLOBAL STATE VARIABLES
// ============================================================================

// Hardware instances
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Bsec iaqSensor;

// Display buffer for formatting text
char oledBuffer[64];

// Application state
AppMode currentAppMode = MODE_OFFLINE;
PowerState currentPowerState = POWER_ACTIVE;
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
ThermalState thermal = THERM_NORMAL;

// Sensor data (validated and filtered)
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

// Configuration values
float seaLevelPressure_hPa_current = 1012.50f;
float gasBaseline_kOhm = NAN;
bool gasBaselineReady = false;

// Timing variables
unsigned long bootMs = 0;
unsigned long lastSensorReadMillis = 0;
unsigned long oledScreenStateChangeMillis = 0;
unsigned long nextSensorRetryMillis = 0;
unsigned long lastBsecSaveMs = 0;
unsigned long nextSafetyProcessMs = 0;

// Retry and error handling
unsigned long sensorRetryBackoffMs = 1000;

// BSEC tracking
bool bsecActive = false;
bool bsecHasData = false;
unsigned long lastBsecDataMs = 0;

// Boot and timeout constants
const unsigned long BOOT_GRACE_MS = 60000UL;
const unsigned long NO_DATA_TIMEOUT_BOOT_MS = 45000UL;
const unsigned long NO_DATA_TIMEOUT_RUN_MS = 20000UL;

// Display change detection (for flicker reduction)
float prev_T = NAN, prev_H = NAN, prev_P = NAN, prev_Alt = NAN;
float prev_G = NAN, prev_IAQ = NAN;
uint8_t prev_Acc = 255;
const char *prev_AQS = "";
unsigned long prevUptimeSec = 0;
OledDisplayState lastDrawnState = OLED_STATE_ERROR_SCREEN;

// Thermal protection
uint8_t overheatPosIndex = 0;

// Filtering state variables
const float IAQ_VAR_ALPHA_LOCAL = IAQ_VAR_ALPHA;
float iaqMeanEWMA = NAN;
float iaqVarEWMA = 0.0f;

uint32_t gasSampleCounter = 0;

const uint8_t PORT_DECIM_N_LOCAL = PORT_DECIM_N;
const uint8_t PORT_BUF_LOCAL = PORT_BUF;
float portBufP[PORT_BUF], portBufH[PORT_BUF], portBufI[PORT_BUF];
uint8_t portFill = 0, portW = 0, portDecim = 0, portStableCount = 0;
bool baselineFrozen = false;

const float ALT_EMA_ALPHA_LOCAL = ALT_EMA_ALPHA;
float gAltSmooth = NAN;
float altRaw3[3] = {NAN, NAN, NAN};
uint8_t altIdx = 0, altCnt = 0;

// ============================================================================
// SECTION 8: UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Inline OLED command helper
 */
static inline void oledCmd(uint8_t c)
{
  display.ssd1306_command(c);
}

/**
 * @brief Set OLED display contrast
 * @param v Contrast value (0-255)
 */
static inline void oledSetContrast(uint8_t v)
{
  oledCmd(SSD1306_SETCONTRAST);
  oledCmd(v);
}

/**
 * @brief Get IAQ category as human-readable string
 * @param x IAQ value (0-500)
 * @return String describing air quality
 */
static const char *getIaqCategory(float x)
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

/**
 * @brief Calculate median of three values
 * @param a First value
 * @param b Second value
 * @param c Third value
 * @return Median value
 */
static inline float med3(float a, float b, float c)
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

/**
 * @brief Calculate barometric altitude from pressure
 * @param press_hPa Current pressure in hPa
 * @param qnh_hPa Sea-level pressure (QNH) in hPa
 * @return Altitude in meters, or NAN if invalid
 */
static inline float simpleBaroAltitude(float press_hPa, float qnh_hPa)
{
  if (!(press_hPa > 0 && qnh_hPa > 0))
    return NAN;
  return 44330.0f * (1.0f - powf(press_hPa / qnh_hPa, 0.190294957f));
}

/**
 * @brief Calculate QNH from known altitude
 * @param press_hPa Current pressure in hPa
 * @param href_m Known altitude in meters
 * @return QNH in hPa, or NAN if invalid
 */
static inline float qnhFromRef(float press_hPa, float href_m)
{
  float k = 1.0f - (href_m / 44330.0f);
  if (k <= 0.0f)
    return NAN;
  return press_hPa / powf(k, 5.255f);
}

/**
 * @brief Check if float value has changed beyond threshold
 * @param current Current value
 * @param previous Previous value
 * @param threshold Minimum change to trigger update
 * @return true if change exceeds threshold
 */
static inline bool hasChanged(float current, float previous, float threshold)
{
  if (isnan(current) || isnan(previous))
    return true;
  return fabsf(current - previous) >= threshold;
}

// ============================================================================
// SECTION 9: OLED DISPLAY FUNCTIONS
// ============================================================================

/**
 * @brief Initialize OLED display
 */
void initOLED()
{
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  oledCmd(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.setTextWrap(false);
  display.display();
  oledSetContrast(CONTRAST_NORMAL);
}

/**
 * @brief Draw thermometer icon for overheat warning
 * @param x X position
 * @param y Y position
 */
void drawThermometerIcon(int16_t x, int16_t y)
{
  display.fillCircle(x + 6, y + 18, 6, SSD1306_WHITE);
  display.fillRect(x + 5, y, 3, 18, SSD1306_WHITE);
}

/**
 * @brief Get animated position for overheat warning
 * @param idx Animation index
 * @param x Output X position
 * @param y Output Y position
 */
void getOverheatPos(uint8_t idx, int16_t &x, int16_t &y)
{
  switch (idx % 5)
  {
  case 0:
    x = 10;
    y = 10;
    break;
  case 1:
    x = 32;
    y = 6;
    break;
  case 2:
    x = 20;
    y = 28;
    break;
  case 3:
    x = 48;
    y = 18;
    break;
  default:
    x = 8;
    y = 36;
    break;
  }
}

/**
 * @brief Display overheat warning with animated icon
 */
void displayOverheat()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&FreeSans12pt7b);

  int16_t x, y;
  getOverheatPos(overheatPosIndex, x, y);
  drawThermometerIcon(x, y);

  int16_t bx, by;
  uint16_t bw, bh;
  display.getTextBounds(F("Overheat"), 0, 0, &bx, &by, &bw, &bh);

  int16_t tx = x + 18, ty = y + 22;
  if (tx + (int)bw > SCREEN_W)
    tx = SCREEN_W - bw;
  if (ty < 18)
    ty = 18;
  if (ty > 56)
    ty = 56;

  display.setCursor(tx, ty);
  display.print(F("Overheat"));
  display.display();

  overheatPosIndex = (overheatPosIndex + 1) % 5;
}

/**
 * @brief Display Screen 1: Temperature & Humidity
 */
void displayScreen1_TempHumid()
{
  const GFXfont *fontHeader = &FreeSans9pt7b;
  const GFXfont *fontData = &FreeSans12pt7b;
  int16_t x1, y1;
  uint16_t w1, h1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setFont(fontHeader);
  display.getTextBounds(F("Temp & Humid"), 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 12);
  display.print(F("Temp & Humid"));
  display.drawFastHLine(0, 15, SCREEN_W, SSD1306_WHITE);

  // Temperature
  display.setFont(fontData);
  snprintf(oledBuffer, sizeof(oledBuffer), "%.2f C", gTemp);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 38);
  display.print(oledBuffer);
  display.drawCircle(((SCREEN_W - w1) / 2) + w1 - 15, 25, 2, SSD1306_WHITE);

  // Humidity
  snprintf(oledBuffer, sizeof(oledBuffer), "%.2f %%", gHum);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 62);
  display.print(oledBuffer);

  display.display();
}

/**
 * @brief Display Screen 2: Pressure & Altitude
 */
void displayScreen2_PressureAlt()
{
  const GFXfont *fontHeader = &FreeSans9pt7b;
  const GFXfont *fontData = &FreeSans12pt7b;
  int16_t x1, y1;
  uint16_t w1, h1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setFont(fontHeader);
  display.getTextBounds(F("Press & Altitude"), 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 12);
  display.print(F("Press & Altitude"));
  display.drawFastHLine(0, 15, SCREEN_W, SSD1306_WHITE);

  // Pressure
  display.setFont(fontData);
  snprintf(oledBuffer, sizeof(oledBuffer), "%.2f hPa", gPress);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 38);
  display.print(oledBuffer);

  // Altitude
  snprintf(oledBuffer, sizeof(oledBuffer), "%d mdpl", (int)lroundf(gAlt));
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 62);
  display.print(oledBuffer);

  display.display();
}

/**
 * @brief Display Screen 3: Gas Resistance & IAQ
 */
void displayScreen3_GasIAQ()
{
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);

  snprintf(oledBuffer, sizeof(oledBuffer), "G: %.1f kOhm", gGasEMA_kOhm);
  display.setCursor(0, 14);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "IAQ: %.1f", gIAQstaticDisp);
  display.setCursor(0, 30);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "Acc: %u", gIAQaccDisp);
  display.setCursor(0, 46);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "AQS: %s", getIaqCategory(gIAQstaticDisp));
  display.setCursor(0, 62);
  display.print(oledBuffer);

  display.display();
}

/**
 * @brief Display Screen 4: System Uptime (HH:MM:SS)
 */
void displayScreen4_Uptime()
{
  const GFXfont *fontHeader = &FreeSans9pt7b;
  const GFXfont *fontData = &FreeSans12pt7b;
  int16_t x1, y1;
  uint16_t w1, h1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setFont(fontHeader);
  display.getTextBounds(F("Uptime"), 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 12);
  display.print(F("Uptime"));
  display.drawFastHLine(0, 15, SCREEN_W, SSD1306_WHITE);

  // Calculate uptime
  unsigned long nowMs = millis();
  unsigned long elapsed = (nowMs >= bootMs) ? (nowMs - bootMs) : nowMs;
  unsigned long totalSec = elapsed / 1000;
  unsigned long h = totalSec / 3600;
  unsigned long m = (totalSec % 3600) / 60;
  unsigned long s = totalSec % 60;

  // Display formatted time
  display.setFont(fontData);
  snprintf(oledBuffer, sizeof(oledBuffer), "%02lu:%02lu:%02lu", h, m, s);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 42);
  display.print(oledBuffer);

  display.display();
}

/**
 * @brief Display error screen when sensor is not available
 */
void displayErrorScreen()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&FreeSans9pt7b);
  display.setCursor(0, 20);
  display.print(F("Sensor Error"));
  display.setFont();
  display.setCursor(0, 40);
  display.print(F("Retrying..."));
  display.display();
}

// ============================================================================
// SECTION 10: DISPLAY STATE MANAGEMENT
// ============================================================================

/**
 * @brief Check if Screen 1 needs redraw (change detection)
 */
bool shouldRedrawScreen1()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  if (isnan(prev_T) || isnan(prev_H))
    return true;
  return hasChanged(gTemp, prev_T, TH_T) || hasChanged(gHum, prev_H, TH_H);
}

/**
 * @brief Check if Screen 2 needs redraw (change detection)
 */
bool shouldRedrawScreen2()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  if (isnan(prev_P) || isnan(prev_Alt))
    return true;
  return hasChanged(gPress, prev_P, TH_P) || hasChanged(gAlt, prev_Alt, TH_ALT);
}

/**
 * @brief Check if Screen 3 needs redraw (change detection)
 */
bool shouldRedrawScreen3()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  const char *aqs = getIaqCategory(gIAQstaticDisp);
  if (isnan(prev_G) || isnan(prev_IAQ))
    return true;
  if (hasChanged(gGasEMA_kOhm, prev_G, TH_G))
    return true;
  if (hasChanged(gIAQstaticDisp, prev_IAQ, TH_IAQ))
    return true;
  if (gIAQaccDisp != prev_Acc)
    return true;
  if (prev_AQS != aqs)
    return true;
  return false;
}

/**
 * @brief Stamp current values as "last drawn" for Screen 1
 */
void stampScreen1()
{
  prev_T = gTemp;
  prev_H = gHum;
  lastDrawnState = OLED_STATE_DATA_SCREEN_1;
}

/**
 * @brief Stamp current values as "last drawn" for Screen 2
 */
void stampScreen2()
{
  prev_P = gPress;
  prev_Alt = gAlt;
  lastDrawnState = OLED_STATE_DATA_SCREEN_2;
}

/**
 * @brief Stamp current values as "last drawn" for Screen 3
 */
void stampScreen3()
{
  prev_G = gGasEMA_kOhm;
  prev_IAQ = gIAQstaticDisp;
  prev_Acc = gIAQaccDisp;
  prev_AQS = getIaqCategory(gIAQstaticDisp);
  lastDrawnState = OLED_STATE_DATA_SCREEN_3;
}

/**
 * @brief Check if Screen 4 needs redraw (uptime change)
 */
bool shouldRedrawScreen4()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  unsigned long nowSec = millis() / 1000;
  return nowSec != prevUptimeSec;
}

/**
 * @brief Stamp current uptime second as "last drawn" for Screen 4
 */
void stampScreen4()
{
  prevUptimeSec = millis() / 1000;
  lastDrawnState = OLED_STATE_DATA_SCREEN_4;
}

/**
 * @brief Update OLED display content based on current state
 */
void updateOLEDDisplayContent()
{
  // Don't update display in error mode
  if (currentAppMode == MODE_BME_ERROR)
  {
    displayErrorScreen();
    return;
  }

  // Handle overheat protection
  if (thermal == THERM_HOT_HOLD)
  {
    if (millis() >= nextSafetyProcessMs)
    {
      displayOverheat();
      nextSafetyProcessMs = millis() + SAFETY_PERIOD_MS;
    }
    return;
  }

  // State machine for screen rotation (continuous loop)
  unsigned long now = millis();
  unsigned long elapsed = now - oledScreenStateChangeMillis;

  switch (currentOledScreenState)
  {
  case OLED_STATE_DATA_SCREEN_1:
    if (elapsed >= OLED_DATA_SCREEN_1_DURATION)
    {
      currentOledScreenState = OLED_STATE_DATA_SCREEN_2;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN; // Force redraw
    }
    else if (shouldRedrawScreen1())
    {
      displayScreen1_TempHumid();
      stampScreen1();
    }
    break;

  case OLED_STATE_DATA_SCREEN_2:
    if (elapsed >= OLED_DATA_SCREEN_2_DURATION)
    {
      currentOledScreenState = OLED_STATE_DATA_SCREEN_3;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN;
    }
    else if (shouldRedrawScreen2())
    {
      displayScreen2_PressureAlt();
      stampScreen2();
    }
    break;

  case OLED_STATE_DATA_SCREEN_3:
    if (elapsed >= OLED_DATA_SCREEN_3_DURATION)
    {
      currentOledScreenState = OLED_STATE_DATA_SCREEN_4;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN;
    }
    else if (shouldRedrawScreen3())
    {
      displayScreen3_GasIAQ();
      stampScreen3();
    }
    break;

  case OLED_STATE_DATA_SCREEN_4:
    if (elapsed >= OLED_DATA_SCREEN_4_DURATION)
    {
      // Loop back to Screen 1 (continuous monitoring)
      currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN;
    }
    else if (shouldRedrawScreen4())
    {
      displayScreen4_Uptime();
      stampScreen4();
    }
    break;

  default:
    // Default to Screen 1
    currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
    oledScreenStateChangeMillis = now;
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
    break;
  }
}

// ============================================================================
// SECTION 11: EEPROM STORAGE MANAGEMENT
// ============================================================================

/**
 * @brief Load BSEC state from EEPROM
 * @return true if state loaded successfully
 */
bool loadBsecState()
{
  uint32_t magic = 0;
  EEPROM.get(BSEC_STATE_VALID_ADDR, magic);
  if (magic != BSEC_STATE_VALID_MAGIC)
    return false;

  uint8_t blob[BSEC_STATE_MAXLEN];
  for (int i = 0; i < BSEC_STATE_MAXLEN; i++)
  {
    EEPROM.get(BSEC_STATE_ADDR + i, blob[i]);
  }
  iaqSensor.setState(blob);
  return true;
}

/**
 * @brief Save BSEC state to EEPROM
 */
void saveBsecState()
{
  uint8_t blob[BSEC_STATE_MAXLEN];
  memset(blob, 0, sizeof(blob));
  iaqSensor.getState(blob);

  for (int i = 0; i < BSEC_STATE_MAXLEN; i++)
  {
    EEPROM.put(BSEC_STATE_ADDR + i, blob[i]);
  }
  uint32_t magic = BSEC_STATE_VALID_MAGIC;
  EEPROM.put(BSEC_STATE_VALID_ADDR, magic);
  EEPROM.commit();
}

/**
 * @brief Load persistent configuration from EEPROM
 */
void loadPersistent()
{
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC_VALUE)
  {
    // First run - initialize EEPROM
    seaLevelPressure_hPa_current = 1012.50f;
    gasBaseline_kOhm = NAN;
    gasBaselineReady = false;

    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.put(GAS_BASELINE_ADDR, gasBaseline_kOhm);
    EEPROM.put(GAS_BASELINE_READY_ADDR, (uint8_t)gasBaselineReady);
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_VALUE);
    EEPROM.commit();
    return;
  }

  // Load sea level pressure with validation
  EEPROM.get(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
  if (isnan(seaLevelPressure_hPa_current) ||
      seaLevelPressure_hPa_current < 870.0f ||
      seaLevelPressure_hPa_current > 1100.0f)
  {
    seaLevelPressure_hPa_current = 1012.50f;
    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.commit();
  }

  // Load gas baseline
  EEPROM.get(GAS_BASELINE_ADDR, gasBaseline_kOhm);
  uint8_t rdy = 0;
  EEPROM.get(GAS_BASELINE_READY_ADDR, rdy);
  gasBaselineReady = (rdy != 0);
}

/**
 * @brief Save sea level pressure to EEPROM
 * @param p Pressure value in hPa
 */
void saveSeaLevelPressure(float p)
{
  EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, p);
  EEPROM.commit();
}

/**
 * @brief Save gas baseline to EEPROM
 * @param b Baseline value
 * @param rdy Ready flag
 */
void saveGasBaseline(float b, bool rdy)
{
  EEPROM.put(GAS_BASELINE_ADDR, b);
  EEPROM.put(GAS_BASELINE_READY_ADDR, (uint8_t)rdy);
  EEPROM.commit();
}

// ============================================================================
// SECTION 12: BME680 & BSEC SENSOR FUNCTIONS
// ============================================================================

/**
 * @brief Initialize BME680 sensor and BSEC library
 * @return true if initialization successful
 */
bool initBSEC()
{
  // Reset BME680 sensor
  Wire.beginTransmission(BME_ADDRESS);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(10);

  // Initialize BSEC
  iaqSensor.begin(BME_ADDRESS, Wire);
  if (iaqSensor.bsecStatus < BSEC_OK || iaqSensor.bme68xStatus != BME68X_OK)
  {
    return false;
  }

  // Subscribe to virtual sensors
  bsec_virtual_sensor_t list[] = {
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_GAS};
  iaqSensor.updateSubscription(list, sizeof(list) / sizeof(list[0]), BSEC_SAMPLE_RATE_LP);
  if (iaqSensor.bsecStatus < BSEC_OK || iaqSensor.bme68xStatus != BME68X_OK)
  {
    return false;
  }

  // Load saved state
  loadBsecState();
  lastBsecSaveMs = millis();
  gIAQaccPrev = iaqSensor.iaqAccuracy;
  gIAQaccDisp = gIAQaccPrev;

  bsecActive = true;
  bsecHasData = false;
  return true;
}

/**
 * @brief Reset altitude filtering state
 */
void resetAltitudeFiltering()
{
  gAltSmooth = NAN;
  altRaw3[0] = NAN;
  altRaw3[1] = NAN;
  altRaw3[2] = NAN;
  altIdx = 0;
  altCnt = 0;
  lastDrawnState = OLED_STATE_ERROR_SCREEN;
}

/**
 * @brief Set QNH (sea-level pressure) value
 * @param qnh QNH value in hPa (870-1100)
 */
void setQNH(float qnh)
{
  if (qnh >= 870.0f && qnh <= 1100.0f)
  {
    seaLevelPressure_hPa_current = qnh;
    saveSeaLevelPressure(qnh);
    resetAltitudeFiltering();
  }
}

/**
 * @brief Calculate QNH from reference altitude
 * @param href_m Reference altitude in meters
 */
void calQNHFromAltRef(float href_m)
{
  if (!(href_m > -1000.0f && href_m < 10000.0f))
    return;
  float q = qnhFromRef(gPress, href_m);
  if (isfinite(q) && q >= 870.0f && q <= 1100.0f)
  {
    setQNH(q);
  }
}

/**
 * @brief Main BSEC processing loop
 * @details Reads sensor data, applies filtering, and updates global state
 */
void bsecLoopTick()
{
  if (!bsecActive)
    return;
  if (!iaqSensor.run())
    return;

  // Read raw sensor data
  gIAQ = iaqSensor.iaq;
  gIAQstatic = iaqSensor.staticIaq;
  gIAQacc = iaqSensor.iaqAccuracy;
  gTemp = iaqSensor.temperature;
  gHum = iaqSensor.humidity;
  gPress = iaqSensor.pressure / 100.0f;
  gGas_kOhm = iaqSensor.gasResistance / 1000.0f;

  // Altitude calculation with median + EMA filtering
  float alt_raw = simpleBaroAltitude(gPress, seaLevelPressure_hPa_current);
  altRaw3[altIdx] = alt_raw;
  altIdx = (altIdx + 1) % 3;
  if (altCnt < 3)
    altCnt++;

  // Median filter for outlier rejection
  float med = (altCnt >= 3) ? med3(altRaw3[0], altRaw3[1], altRaw3[2]) : alt_raw;
  if (fabsf(alt_raw - med) > ALT_OUTLIER_M)
  {
    alt_raw = med;
  }

  // EMA with deadband
  if (isnan(gAltSmooth))
  {
    gAltSmooth = alt_raw;
  }
  else
  {
    float cand = ALT_EMA_ALPHA_LOCAL * alt_raw + (1.0f - ALT_EMA_ALPHA_LOCAL) * gAltSmooth;
    if (fabsf(cand - gAltSmooth) >= ALT_DEADBAND_M)
    {
      gAltSmooth = cand;
    }
  }
  gAlt = gAltSmooth;

  // Mark BSEC data as available
  if (!bsecHasData)
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
  bsecHasData = true;
  lastBsecDataMs = millis();

  // Gas resistance EMA filtering
  if (isnan(gGasEMA_kOhm))
  {
    gGasEMA_kOhm = gGas_kOhm;
  }
  else
  {
    gGasEMA_kOhm = GAS_EMA_ALPHA * gGas_kOhm + (1.0f - GAS_EMA_ALPHA) * gGasEMA_kOhm;
  }

  // IAQ variance-adaptive smoothing
  if (isnan(iaqMeanEWMA))
  {
    iaqMeanEWMA = gIAQstatic;
    iaqVarEWMA = 0.0f;
  }
  float d = gIAQstatic - iaqMeanEWMA;
  iaqMeanEWMA += IAQ_VAR_ALPHA_LOCAL * d;
  iaqVarEWMA = (1.0f - IAQ_VAR_ALPHA_LOCAL) * (iaqVarEWMA + IAQ_VAR_ALPHA_LOCAL * d * d);
  float vol = sqrtf(fmaxf(iaqVarEWMA, 0.0f));

  // Adaptive smoothing based on variance
  float aVar;
  if (vol <= 2.0f)
    aVar = 0.10f;
  else if (vol >= 25.0f)
    aVar = 0.45f;
  else
    aVar = 0.10f + (vol - 2.0f) * (0.35f / (25.0f - 2.0f));

  // Adjust based on accuracy
  if (gIAQacc >= 3)
    aVar *= 0.6f;
  if (gIAQacc <= 1)
    aVar = fmaxf(aVar, 0.28f);
  aVar = fminf(fmaxf(aVar, 0.08f), 0.45f);

  // Apply IAQ smoothing
  if (isnan(gIAQstaticDisp))
  {
    gIAQstaticDisp = gIAQstatic;
  }
  else
  {
    gIAQstaticDisp = aVar * gIAQstatic + (1.0f - aVar) * gIAQstaticDisp;
  }

  // Periodic BSEC state save (every 4 hours when accuracy is high)
  if (millis() - lastBsecSaveMs >= BSEC_SAVE_INTERVAL_MS && gIAQacc >= 3)
  {
    saveBsecState();
    lastBsecSaveMs = millis();
  }

  // Save on accuracy improvement
  if (gIAQacc > gIAQaccPrev && gIAQacc >= 2)
  {
    if (millis() - lastBsecSaveMs >= BSEC_MIN_SAVE_GAP_MS)
    {
      saveBsecState();
      lastBsecSaveMs = millis();
    }
  }

  // Accuracy display (hysteresis to prevent flickering)
  gIAQaccDisp = (gIAQacc >= 3) ? 3 : (gIAQacc == 2 && gIAQaccDisp == 3 ? 2 : gIAQacc);
  gIAQaccPrev = gIAQacc;

  // Transport detection (freeze baseline during movement)
  if (++portDecim >= PORT_DECIM_N_LOCAL)
  {
    portDecim = 0;
    portBufP[portW] = gPress;
    portBufH[portW] = gHum;
    portBufI[portW] = gIAQstatic;
    portW = (portW + 1) % PORT_BUF_LOCAL;
    if (portFill < PORT_BUF_LOCAL)
      portFill++;

    if (portFill >= 2)
    {
      int newest = (portW + PORT_BUF_LOCAL - 1) % PORT_BUF_LOCAL;
      int oldest = (portFill == PORT_BUF_LOCAL) ? portW : 0;
      float dP = portBufP[newest] - portBufP[oldest];
      float dH = portBufH[newest] - portBufH[oldest];
      float dI = portBufI[newest] - portBufI[oldest];
      bool moving = (fabsf(dP) > PORT_TH_DP) || (fabsf(dH) > PORT_TH_DH) || (fabsf(dI) > PORT_TH_DIAQ);

      if (moving)
      {
        baselineFrozen = true;
        portStableCount = 0;
      }
      else
      {
        if (baselineFrozen)
        {
          if (++portStableCount >= PORT_RESUME_OK)
          {
            baselineFrozen = false;
          }
        }
      }
    }
  }
}

/**
 * @brief Read and process BME680 sensor data
 */
void readBME680SensorData()
{
  if (!bsecActive)
    return;

  // Check if BSEC has recent data
  unsigned long noDataTimeout = (millis() - bootMs < BOOT_GRACE_MS) ? NO_DATA_TIMEOUT_BOOT_MS : NO_DATA_TIMEOUT_RUN_MS;
  if (!bsecHasData || (millis() - lastBsecDataMs > noDataTimeout))
  {
    currentAppMode = MODE_BME_ERROR;
    display.clearDisplay();
    display.display();
    sensorRetryBackoffMs = 1000;
    nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
    return;
  }

  // Adaptive gas baseline tracking (only when not frozen)
  if (!baselineFrozen)
  {
    if (isnan(gasBaseline_kOhm))
    {
      gasBaseline_kOhm = gGasEMA_kOhm;
    }
    float dv = gGasEMA_kOhm - gasBaseline_kOhm;
    gasBaseline_kOhm += (dv > 0 ? BASELINE_ALPHA_UP : BASELINE_ALPHA_DOWN) * dv;
  }

  gasSampleCounter++;
  if (!gasBaselineReady && gasSampleCounter >= GAS_BASELINE_READY_SAMPLES)
  {
    gasBaselineReady = true;
    saveGasBaseline(gasBaseline_kOhm, true);
  }

  // Thermal protection check
  if (thermal == THERM_NORMAL && gTemp >= HOT_ENTER_C)
  {
    enterHotHold();
    return;
  }
  if (thermal == THERM_HOT_HOLD && gTemp <= HOT_EXIT_C)
  {
    exitHotHold();
  }

  lastDrawnState = OLED_STATE_ERROR_SCREEN;
}

// ============================================================================
// SECTION 13: THERMAL PROTECTION
// ============================================================================

/**
 * @brief Enter overheat protection mode
 */
void enterHotHold()
{
  thermal = THERM_HOT_HOLD;
  oledSetContrast(CONTRAST_OVERHEAT);
  displayOverheat();
  nextSafetyProcessMs = millis();
}

/**
 * @brief Exit overheat protection mode
 */
void exitHotHold()
{
  thermal = THERM_NORMAL;
  oledSetContrast(CONTRAST_NORMAL);
  lastDrawnState = OLED_STATE_ERROR_SCREEN;
}

// ============================================================================
// SECTION 14: ERROR RECOVERY & RETRY
// ============================================================================

/**
 * @Schedule initial sensor retry
 */
void scheduleSensorRetryInitial()
{
  sensorRetryBackoffMs = 1000;
  nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
}

/**
 * @brief Handle sensor auto-retry with exponential backoff
 */
void handleSensorAutoRetry()
{
  if (millis() < nextSensorRetryMillis)
    return;

  if (initBSEC())
  {
    currentAppMode = MODE_OFFLINE;
    bsecActive = true;
    bsecHasData = false;
    display.clearDisplay();
    display.display();
    delay(200);
    lastSensorReadMillis = millis() - SENSOR_READ_INTERVAL_MS;
    oledScreenStateChangeMillis = millis();
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
    return;
  }

  // Exponential backoff (max 60 seconds)
  if (sensorRetryBackoffMs < 60000)
    sensorRetryBackoffMs *= 2;
  if (sensorRetryBackoffMs > 60000)
    sensorRetryBackoffMs = 60000;
  nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
}

// ============================================================================
// SECTION 15: SERIAL COMMAND INTERFACE
// ============================================================================

/**
 * @brief Handle serial input commands
 */
void handleSerialInput()
{
  static String line;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      line.trim();
      if (line.length())
      {
        if (line.startsWith("QNH="))
        {
          setQNH(line.substring(4).toFloat());
          Serial.printf("OK QNH=%.2f hPa\r\n", seaLevelPressure_hPa_current);
        }
        else if (line.equalsIgnoreCase("QNH?"))
          Serial.printf("QNH=%.2f hPa\r\n", seaLevelPressure_hPa_current);
        else if (line.equalsIgnoreCase("ALT?"))
          Serial.printf("ALT=%.2f m\r\n", gAlt);
        else if (line.equalsIgnoreCase("PRESS?"))
          Serial.printf("P=%.2f hPa\r\n", gPress);
        else if (line.startsWith("ALTREF="))
        {
          calQNHFromAltRef(line.substring(7).toFloat());
          Serial.printf("OK QNH=%.2f hPa\r\n", seaLevelPressure_hPa_current);
        }
        else if (line.equalsIgnoreCase("STATUS"))
          Serial.printf("Mode:%d Therm:%d BSEC:%s IAQ:%.1f(%u)\r\n",
                        currentAppMode, thermal, bsecActive ? "Y" : "N",
                        gIAQstaticDisp, gIAQaccDisp);
        else if (line.equalsIgnoreCase("HELP"))
          Serial.println(F("CMD: QNH=<hPa>|QNH?|ALT?|PRESS?|ALTREF=<m>|STATUS|HELP"));
      }
      line = "";
    }
    else
    {
      line += c;
    }
  }
}

// ============================================================================
// SECTION 16: SETUP & MAIN LOOP
// ============================================================================

/**
 * @brief Arduino setup function
 */
void setup()
{
  Serial.begin(115200);
  Serial.println(F("\r\n=== ESP8266 BME680 Monitor v2.0 ==="));

  bootMs = millis();

  EEPROM.begin(EEPROM_SIZE);
  loadPersistent();

  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
  Wire.setClock(I2C_CLOCK_SLOW);
  delay(80);

  initOLED();

  if (!initBSEC())
  {
    currentAppMode = MODE_BME_ERROR;
    sensorRetryBackoffMs = 1000;
    nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
    Serial.println(F("BSEC: init failed, will retry"));
  }
  else
  {
    Serial.println(F("BSEC: ready"));
  }

  Wire.setClock(I2C_CLOCK_FAST);
  lastSensorReadMillis = millis();
  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
  oledCmd(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.display();
  lastDrawnState = OLED_STATE_ERROR_SCREEN;

  Serial.println(F("Type HELP for commands"));
}

/**
 * @brief Arduino main loop
 */
void loop()
{
  // Process BSEC data (must be called regularly)
  bsecLoopTick();

  // Main state machine
  if (currentAppMode == MODE_OFFLINE)
  {
    // Check thermal protection
    if (thermal == THERM_HOT_HOLD)
    {
      if (millis() >= nextSafetyProcessMs)
      {
        if (gTemp <= HOT_EXIT_C)
        {
          exitHotHold();
        }
        if (thermal == THERM_HOT_HOLD)
        {
          displayOverheat();
        }
        nextSafetyProcessMs = millis() + SAFETY_PERIOD_MS;
      }
    }
    else
    {
      // Normal sensor reading cycle
      if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS)
      {
        readBME680SensorData();
        lastSensorReadMillis = millis();
      }

      // Update display
      updateOLEDDisplayContent();
    }
  }
  else if (currentAppMode == MODE_BME_ERROR)
  {
    // Handle sensor recovery
    handleSensorAutoRetry();
  }

  // Process serial commands
  handleSerialInput();

  // Allow ESP8266 background tasks
  yield();
}
