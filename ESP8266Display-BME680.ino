#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SSD1306Wire.h>
#include <EEPROM.h>
// #include <ESP8266WiFi.h>  // Uncomment if you want optional light-sleep hook

// -------- I2C & device address --------
#define OLED_ADDRESS 0x3C
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define BME_ADDRESS  0x76

// -------- Timing --------
const unsigned long SENSOR_READ_INTERVAL_MS = 30000;
const unsigned long OLED_DATA_SCREEN_1_DURATION_MS = 7000;
const unsigned long OLED_DATA_SCREEN_2_DURATION_MS = 3000;
const unsigned long OLED_OFF_DURATION_MS =
    SENSOR_READ_INTERVAL_MS - (OLED_DATA_SCREEN_1_DURATION_MS + OLED_DATA_SCREEN_2_DURATION_MS);

// -------- BME680 robustness --------
const uint8_t  BME680_RELIABLE_READ_RETRIES = 5;
const uint16_t BME680_RELIABLE_READ_RETRY_DELAY_MS = 50;
const uint16_t BME680_INIT_RETRY_INTERVAL_MS = 1500;
const uint8_t  BME680_MAX_INIT_ATTEMPTS = 15;  // faster fail → error mode

// -------- EEPROM layout --------
#define EEPROM_SIZE 512
#define SEA_LEVEL_PRESSURE_ADDR 0           // float (4B)
#define GAS_BASELINE_ADDR       4           // float (4B)
#define GAS_BASELINE_READY_ADDR 8           // uint8_t (1B)
#define EEPROM_MAGIC_ADDR       9           // uint32_t (4B)
#define EEPROM_MAGIC_VALUE      0xB6680A11  // simple guard

// -------- Display & sensor --------
SSD1306Wire display(OLED_ADDRESS, I2C_SDA_GPIO, I2C_SCL_GPIO);
Adafruit_BME680 bme(&Wire);

// -------- Buffers & globals --------
char   oledBuffer[64];

float gTemp = 0.0f;
float gHum = 0.0f;
float gPress = 0.0f;        // hPa
float gAlt = 0.0f;
float gGas_kOhm = 0.0f;     // kΩ (converted)
float gGasEMA_kOhm = NAN;   // smoothed gas
float seaLevelPressure_hPa_current = 1012.50f;

float gasBaseline_kOhm = NAN;   // local "p95-like" baseline
bool  gasBaselineReady  = false;

enum AppMode { MODE_OFFLINE, MODE_BME_ERROR };
AppMode currentAppMode = MODE_OFFLINE;

enum OledDisplayState { OLED_STATE_ERROR_SCREEN, OLED_STATE_DATA_SCREEN_1, OLED_STATE_DATA_SCREEN_2, OLED_STATE_OFF };
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;

unsigned long lastSensorReadMillis = 0;
unsigned long oledScreenStateChangeMillis = 0;

// -------- IAQ feel parameters --------
// EMA smoothing (choose 0.1–0.3); higher = faster response
const float GAS_EMA_ALPHA = 0.20f;

// Baseline estimator: rise faster when above baseline, decay very slow.
// This approximates a high quantile (~p95) without storing history.
const float BASELINE_ALPHA_UP   = 0.02f;   // raise towards new highs
const float BASELINE_ALPHA_DOWN = 0.001f;  // decay very slowly

// Relative thresholds against baseline (higher kΩ = better air)
const float TH_VERY_POOR = 0.60f;
const float TH_POOR      = 0.75f;
const float TH_NORMAL    = 0.90f;
const float TH_GOOD      = 1.05f;  // >= 1.05*baseline ⇒ Excellent

// Hysteresis in absolute kΩ to avoid flip-flop (per your request)
const float STATUS_HYST_KOHM = 3.0f;

// Count towards "24h warmup": we don't strictly need 24h persistence,
// but we keep a long-running baseline and mark "ready" after enough samples.
// 24h @30s = 2880; we won't store this in EEPROM to avoid wear.
uint32_t gasSampleCounter = 0;
const uint32_t GAS_BASELINE_READY_SAMPLES = 2880;

// -------- Forward decl. --------
void initOLED();
void initBME680();
bool selfTestBME680();
void readBME680SensorData();
bool readBME680SensorReliably(float &temp, float &hum, float &press_Pa, float &alt, float &gas_kOhm);
void updateOLEDDisplayContent();
void displayCenteredStatus(const String& line1, const String& line2);
void displaySensorDataScreen1();
void displaySensorDataScreen2();
String getGasStatus(float gas_kOhm);
String getGasStatusWithHysteresis(float gas_kOhm);
void loadPersistent();
void saveSeaLevelPressure(float pressure);
void saveGasBaseline(float baseline, bool ready);
void handleSerialInput();
// Optional power hook without GPIO16 (kept disabled by default)
void lightSleepDuringOff(unsigned long ms);

enum GasStatus { VS_VERY_POOR, VS_POOR, VS_NORMAL, VS_GOOD, VS_EXCELLENT };
GasStatus lastStatus = VS_NORMAL;

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  loadPersistent();

  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
  Wire.setClock(400000);  // try fast I2C

  initOLED();
  displayCenteredStatus("BME680 Monitor", "Initializing...");
  delay(500);

  initBME680();
  if (!selfTestBME680()) {
    // Fallback to standard clock and retry once
    Wire.setClock(100000);
    initBME680();
    if (!selfTestBME680()) {
      currentAppMode = MODE_BME_ERROR;
      displayCenteredStatus("BME680 Fail!", "Error Mode");
    }
  }

  lastSensorReadMillis = millis();
  oledScreenStateChangeMillis = millis();
  if (currentAppMode == MODE_OFFLINE) readBME680SensorData();
}

// -------------------- Loop --------------------
void loop() {
  if (currentAppMode == MODE_OFFLINE) {
    if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
      readBME680SensorData();
      lastSensorReadMillis = millis();
    }
  } else {
    // Error mode: keep UI responsive
    yield();
  }

  updateOLEDDisplayContent();
  handleSerialInput();
  yield();
}

// -------------------- OLED --------------------
void initOLED() {
  if (!display.init()) {
    while (1) { yield(); } // hard stop if OLED missing
  }
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
}

void displayCenteredStatus(const String& line1, const String& line2) {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  snprintf(oledBuffer, sizeof(oledBuffer), "%s", line1.c_str());
  display.drawString(64, 15, oledBuffer);
  snprintf(oledBuffer, sizeof(oledBuffer), "%s", line2.c_str());
  display.drawString(64, 40, oledBuffer);
  display.display();
}

void displaySensorDataScreen1() {
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  snprintf(oledBuffer, sizeof(oledBuffer), "Temp: %.2f C", gTemp);
  display.drawString(0, 0, oledBuffer);
  snprintf(oledBuffer, sizeof(oledBuffer), "Hum: %.2f %%", gHum);
  display.drawString(0, 16, oledBuffer);
  snprintf(oledBuffer, sizeof(oledBuffer), "Press: %.1f hPa", gPress);
  display.drawString(0, 32, oledBuffer);
  snprintf(oledBuffer, sizeof(oledBuffer), "Alti: %.2f m", gAlt);
  display.drawString(0, 48, oledBuffer);
}

void displaySensorDataScreen2() {
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  snprintf(oledBuffer, sizeof(oledBuffer), "Gas: %.2f kOhm", gGasEMA_kOhm);
  display.drawString(0, 0, oledBuffer);

  display.drawString(0, 16, "Status:");
  String st = getGasStatusWithHysteresis(gGasEMA_kOhm);
  display.drawString(0, 32, st);
}

void updateOLEDDisplayContent() {
  if (currentAppMode == MODE_BME_ERROR) return;

  unsigned long now = millis();
  unsigned long elapsed = now - oledScreenStateChangeMillis;

  switch (currentOledScreenState) {
    case OLED_STATE_DATA_SCREEN_1:
      if (elapsed >= OLED_DATA_SCREEN_1_DURATION_MS) {
        currentOledScreenState = OLED_STATE_DATA_SCREEN_2;
        oledScreenStateChangeMillis = now;
      }
      break;
    case OLED_STATE_DATA_SCREEN_2:
      if (elapsed >= OLED_DATA_SCREEN_2_DURATION_MS) {
        currentOledScreenState = OLED_STATE_OFF;
        oledScreenStateChangeMillis = now;
        display.displayOff();   // turn panel off immediately
        // lightSleepDuringOff(OLED_OFF_DURATION_MS); // optional low-current idle
      }
      break;
    case OLED_STATE_OFF:
      if (elapsed >= OLED_OFF_DURATION_MS) {
        currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
        oledScreenStateChangeMillis = now;
        display.displayOn();
        delay(50); // small panel wake-up guard
      } else {
        // Do nothing while off (no buffer push)
        return;
      }
      break;
    case OLED_STATE_ERROR_SCREEN:
      // not used in normal flow
      break;
  }

  display.clear();
  if (currentOledScreenState == OLED_STATE_DATA_SCREEN_1) {
    displaySensorDataScreen1();
    display.display();
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_2) {
    displaySensorDataScreen2();
    display.display();
  } else if (currentOledScreenState == OLED_STATE_ERROR_SCREEN) {
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 15, "Sensor Error");
    display.drawString(64, 40, "Check Conn.!");
    display.display();
  }
}

// -------------------- BME680 --------------------
void initBME680() {
  uint8_t attempts = 0;
  while (!bme.begin(BME_ADDRESS)) {
    attempts++;
    displayCenteredStatus("BME680 Init Fail!", "Retry " + String(attempts));
    delay(BME680_INIT_RETRY_INTERVAL_MS);
    if (attempts >= BME680_MAX_INIT_ATTEMPTS) {
      currentAppMode = MODE_BME_ERROR;
      displayCenteredStatus("BME680 Fail!", "Error Mode");
      return;
    }
  }

  // Oversampling & filter tuned for good accuracy at 30s cadence
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  // Gas heater pulse (ULP-like)
  bme.setGasHeater(320, 150);  // 320°C for 150ms
}

bool selfTestBME680() {
  // Quick read & range sanity check
  for (uint8_t i = 0; i < 3; i++) {
    if (bme.performReading()) {
      float T = bme.temperature;
      float H = bme.humidity;
      float P = bme.pressure;
      if (!isnan(T) && !isnan(H) && !isnan(P)) return true;
    }
    delay(100);
  }
  return false;
}

void readBME680SensorData() {
  float t, h, p_Pa, alt, gas_k;
  if (!readBME680SensorReliably(t, h, p_Pa, alt, gas_k)) {
    currentAppMode = MODE_BME_ERROR;
    displayCenteredStatus("Sensor Read Fail!", "Check Conn.!");
    return;
  }

  gTemp = t;
  gHum = h;
  gPress = p_Pa / 100.0f; // Pa → hPa
  gAlt = alt;
  gGas_kOhm = gas_k;

  // EMA smoothing on gas
  if (isnan(gGasEMA_kOhm)) gGasEMA_kOhm = gGas_kOhm;
  else gGasEMA_kOhm = GAS_EMA_ALPHA * gGas_kOhm + (1.0f - GAS_EMA_ALPHA) * gGasEMA_kOhm;

  // Baseline estimator (high-quantile-like)
  if (isnan(gasBaseline_kOhm)) gasBaseline_kOhm = gGasEMA_kOhm;
  float delta = gGasEMA_kOhm - gasBaseline_kOhm;
  if (delta > 0) gasBaseline_kOhm += BASELINE_ALPHA_UP * delta;
  else           gasBaseline_kOhm += BASELINE_ALPHA_DOWN * delta;

  gasSampleCounter++;
  if (!gasBaselineReady && gasSampleCounter >= GAS_BASELINE_READY_SAMPLES) {
    gasBaselineReady = true;
    saveGasBaseline(gasBaseline_kOhm, gasBaselineReady); // store once
  }

  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
  display.displayOn(); // ensure on when new data arrives
  delay(50);
}

bool readBME680SensorReliably(float &temp, float &hum, float &press_Pa, float &alt, float &gas_kOhm) {
  for (uint8_t i = 0; i < BME680_RELIABLE_READ_RETRIES; i++) {
    if (bme.performReading()) {
      temp      = bme.temperature;
      hum       = bme.humidity;
      press_Pa  = bme.pressure;
      alt       = bme.readAltitude(seaLevelPressure_hPa_current);
      gas_kOhm  = bme.gas_resistance / 1000.0f;

      if (!isnan(temp) && !isnan(hum) && !isnan(press_Pa) && !isnan(alt) && !isnan(gas_kOhm) &&
          temp >= -40.0f && temp <= 85.0f &&
          hum  >=   0.0f && hum  <= 100.0f &&
          press_Pa >= 30000.0f && press_Pa <= 110000.0f &&
          gas_kOhm >= 0.0f) {
        return true;
      }
    }
    delay(BME680_RELIABLE_READ_RETRY_DELAY_MS);
  }
  return false;
}

// -------------------- IAQ label with hysteresis --------------------
String getGasStatusWithHysteresis(float gas_kOhm) {
  // Use relative thresholds against baseline, then apply ±3 kΩ hysteresis.
  if (isnan(gasBaseline_kOhm) || gasBaseline_kOhm < 1.0f) return "Calibrating";

  float r = gas_kOhm / gasBaseline_kOhm;
  GasStatus next = lastStatus;

  switch (lastStatus) {
    case VS_VERY_POOR:
      if (r >= TH_POOR || gas_kOhm >= (TH_POOR * gasBaseline_kOhm + STATUS_HYST_KOHM)) next = VS_POOR;
      break;
    case VS_POOR:
      if (r < TH_VERY_POOR && gas_kOhm < (TH_VERY_POOR * gasBaseline_kOhm - STATUS_HYST_KOHM)) next = VS_VERY_POOR;
      else if (r >= TH_NORMAL || gas_kOhm >= (TH_NORMAL * gasBaseline_kOhm + STATUS_HYST_KOHM)) next = VS_NORMAL;
      break;
    case VS_NORMAL:
      if (r < TH_POOR && gas_kOhm < (TH_POOR * gasBaseline_kOhm - STATUS_HYST_KOHM)) next = VS_POOR;
      else if (r >= TH_GOOD || gas_kOhm >= (TH_GOOD * gasBaseline_kOhm + STATUS_HYST_KOHM)) next = VS_GOOD;
      break;
    case VS_GOOD:
      if (r < TH_NORMAL && gas_kOhm < (TH_NORMAL * gasBaseline_kOhm - STATUS_HYST_KOHM)) next = VS_NORMAL;
      else if (r >= 1.10f || gas_kOhm >= (1.10f * gasBaseline_kOhm + STATUS_HYST_KOHM)) next = VS_EXCELLENT;
      break;
    case VS_EXCELLENT:
      if (r < TH_GOOD && gas_kOhm < (TH_GOOD * gasBaseline_kOhm - STATUS_HYST_KOHM)) next = VS_GOOD;
      break;
  }

  lastStatus = next;
  return getGasStatus(gas_kOhm); // reuse text mapping
}

String getGasStatus(float gas_kOhm) {
  if (isnan(gasBaseline_kOhm) || gasBaseline_kOhm < 1.0f) return "Calibrating";

  float r = gas_kOhm / gasBaseline_kOhm;
  if (r <= TH_VERY_POOR) return "Very Poor";
  else if (r <= TH_POOR) return "Poor";
  else if (r <= TH_NORMAL) return "Normal";
  else if (r <= TH_GOOD) return "Good";
  else return "Excellent";
}

// -------------------- Persistence --------------------
void loadPersistent() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC_VALUE) {
    // Fresh EEPROM: write defaults
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

  EEPROM.get(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
  if (isnan(seaLevelPressure_hPa_current) ||
      seaLevelPressure_hPa_current < 870.0f || seaLevelPressure_hPa_current > 1100.0f) {
    seaLevelPressure_hPa_current = 1012.50f;
    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.commit();
  }

  EEPROM.get(GAS_BASELINE_ADDR, gasBaseline_kOhm);
  uint8_t rdy = 0;
  EEPROM.get(GAS_BASELINE_READY_ADDR, rdy);
  gasBaselineReady = (rdy != 0);
}

void saveSeaLevelPressure(float pressure) {
  EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, pressure);
  EEPROM.commit();
}

void saveGasBaseline(float baseline, bool ready) {
  EEPROM.put(GAS_BASELINE_ADDR, baseline);
  EEPROM.put(GAS_BASELINE_READY_ADDR, (uint8_t)ready);
  EEPROM.commit();
}

// -------------------- Serial command --------------------
void handleSerialInput() {
  static String cmd = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmd.trim();
      if (cmd.startsWith("setpressure ")) {
        String s = cmd.substring(12);
        float n = s.toFloat();
        if (n >= 300.0f && n <= 1100.0f) {
          seaLevelPressure_hPa_current = n;
          saveSeaLevelPressure(seaLevelPressure_hPa_current);
          Serial.print("Pressure Updated! ");
          Serial.println(String(seaLevelPressure_hPa_current, 2) + " hPa");
          snprintf(oledBuffer, sizeof(oledBuffer), "%.2f", seaLevelPressure_hPa_current);
          displayCenteredStatus("Pressure Updated!", oledBuffer);
          delay(1200);
          oledScreenStateChangeMillis = millis();
          currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
        }
      } else if (cmd == "getbaseline") {
        Serial.print("Baseline(kOhm): ");
        Serial.println(isnan(gasBaseline_kOhm) ? String("NaN") : String(gasBaseline_kOhm, 2));
        Serial.print("BaselineReady: ");
        Serial.println(gasBaselineReady ? "true" : "false");
      } else if (cmd == "resetbaseline") {
        gasBaseline_kOhm = NAN;
        gasBaselineReady = false;
        saveGasBaseline(gasBaseline_kOhm, gasBaselineReady);
        Serial.println("Baseline reset.");
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }
}

// -------------------- Optional light sleep (no GPIO16) --------------------
void lightSleepDuringOff(unsigned long ms) {
  // Use only if WiFi lib included and you accept blocking sleep during OLED OFF
  // WiFi.mode(WIFI_OFF);
  // WiFi.forceSleepBegin();
  // delay(ms);
  // WiFi.forceSleepWake();
}
