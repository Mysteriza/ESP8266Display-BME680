#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <bsec.h>
#include <math.h>

// -------- I2C / OLED / BME --------
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define OLED_ADDRESS 0x3C
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_RESET -1
#define BME_ADDRESS  0x76

// -------- UI timing --------
const unsigned long SENSOR_READ_INTERVAL_MS = 30000;
const unsigned long OLED_DATA_SCREEN_1_DURATION_MS = 5000;
const unsigned long OLED_DATA_SCREEN_2_DURATION_MS = 5000;
const unsigned long OLED_OFF_DURATION_MS =
  SENSOR_READ_INTERVAL_MS - (OLED_DATA_SCREEN_1_DURATION_MS + OLED_DATA_SCREEN_2_DURATION_MS);

// -------- EEPROM layout --------
#define EEPROM_SIZE 512
#define SEA_LEVEL_PRESSURE_ADDR 0
#define GAS_BASELINE_ADDR       4
#define GAS_BASELINE_READY_ADDR 8
#define EEPROM_MAGIC_ADDR       9
#define EEPROM_MAGIC_VALUE      0xB6680A11
#define BSEC_STATE_ADDR         16
#define BSEC_STATE_MAXLEN       BSEC_MAX_STATE_BLOB_SIZE
#define BSEC_STATE_VALID_ADDR   (BSEC_STATE_ADDR + BSEC_STATE_MAXLEN)
#define BSEC_STATE_VALID_MAGIC  0xB5EC1A0F

// -------- QNH (Open-Meteo) --------
#define ENABLE_BOOT_QNH_WIFI          1
#define ENABLE_PERIODIC_QNH_REFRESH   1
#define SHOW_QNH_UPDATED_ON_PERIODIC  1
#define QNH_UPDATED_DISPLAY_MS        2000

const char* WIFI_SSID = "Kosan bu nata";
const char* WIFI_PASS = "immodium";
const float OM_LAT = -6.914744f;
const float OM_LON = 107.609810f;

const uint16_t WIFI_CONNECT_TIMEOUT_MS = 6000;
const uint16_t QNH_FETCH_TIMEOUT_MS    = 3000;
const unsigned long QNH_REFRESH_INTERVAL_MS = 3UL*60UL*60UL*1000UL;

// -------- Devices --------
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);
Adafruit_BME680  bme(&Wire);   // fallback
Bsec             iaqSensor;    // BSEC

// -------- Globals --------
char   oledBuffer[64];
float gTemp=0, gHum=0, gPress=0, gAlt=0, gGas_kOhm=0, gGasEMA_kOhm=NAN;
float gIAQ= NAN, gIAQstatic = NAN; uint8_t gIAQacc = 0, gIAQaccPrev = 0;
float seaLevelPressure_hPa_current = 1012.50f;

float gasBaseline_kOhm = NAN; bool gasBaselineReady=false;

enum AppMode { MODE_OFFLINE, MODE_BME_ERROR };
AppMode currentAppMode = MODE_OFFLINE;

enum OledDisplayState { OLED_STATE_ERROR_SCREEN, OLED_STATE_DATA_SCREEN_1, OLED_STATE_DATA_SCREEN_2, OLED_STATE_OFF };
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;

unsigned long lastSensorReadMillis=0, oledScreenStateChangeMillis=0;
#if ENABLE_PERIODIC_QNH_REFRESH
unsigned long nextQNHFetchMillis=0;
#endif

// IAQ feel (local label)
const float GAS_EMA_ALPHA=0.20f;
const float BASELINE_ALPHA_UP=0.02f, BASELINE_ALPHA_DOWN=0.001f;
const float TH_VERY_POOR=0.60f, TH_POOR=0.75f, TH_NORMAL=0.90f, TH_GOOD=1.05f;
const float STATUS_HYST_KOHM=3.0f;
uint32_t gasSampleCounter=0;
const uint32_t GAS_BASELINE_READY_SAMPLES=2880;
enum GasStatus { VS_VERY_POOR, VS_POOR, VS_NORMAL, VS_GOOD, VS_EXCELLENT };
GasStatus lastStatus = VS_NORMAL;

// Auto-retry
unsigned long nextSensorRetryMillis = 0;
unsigned long sensorRetryBackoffMs  = 1000;

// BSEC state save
unsigned long lastBsecSaveMs = 0;
const unsigned long BSEC_SAVE_INTERVAL_MS = 2UL*60UL*60UL*1000UL; // 2h baseline
const unsigned long BSEC_MIN_SAVE_GAP_MS  = 10UL*60UL*1000UL;     // 10 min (event-based)

// BSEC loop flags
bool bsecActive = false;
bool bsecHasData = false;
unsigned long lastBsecDataMs = 0;

// -------- Fwd decl --------
void initOLED();
void displayCenteredStatus(const String&, const String&);
void displaySensorDataScreen1();
void displaySensorDataScreen2();
void updateOLEDDisplayContent();

bool initBSEC();
void bsecLoopTick();
void readBME680SensorData();

void initBME680(); bool selfTestBME680();
void scheduleSensorRetryInitial(); void handleSensorAutoRetry();

void loadPersistent(); void saveSeaLevelPressure(float p);
void saveGasBaseline(float b,bool rdy);
bool loadBsecState(); void saveBsecState();

void handleSerialInput();
void lightSleepDuringOff(unsigned long ms);

// Open-Meteo helpers
static void omBuildPathCurrent(String& out);
static void omBuildPathHourly(String& out);
static bool extractPressureMSL_Current(const String& body, float& val);
static bool extractPressureMSL_Hourly_Last(const String& body, float& val);
static bool httpGetOpenMeteo(const String& path, String& resp);
static bool fetchQNHfromOpenMeteo(float &out_hPa);

// IAQ buckets (Bosch-like)
static String iaqCategory(float x){
  if (isnan(x)) return "n/a";
  if (x <=  50) return "Excellent";
  if (x <= 100) return "Good";
  if (x <= 150) return "Light";
  if (x <= 200) return "Moderate";
  if (x <= 300) return "Unhealthy";
  return "Hazardous";
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200); // minimal CLI
  EEPROM.begin(EEPROM_SIZE);
  loadPersistent();

  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
  Wire.setClock(400000);

  initOLED();
  displayCenteredStatus("BME680 Monitor","Initializing...");
  delay(500);

  if (!initBSEC()){
    initBME680();
    if (!selfTestBME680()){
      currentAppMode = MODE_BME_ERROR;
      displayCenteredStatus("BSEC/BME Fail!","Error Mode");
      scheduleSensorRetryInitial();
    }
  }

#if ENABLE_BOOT_QNH_WIFI
  if (currentAppMode == MODE_OFFLINE) {
    float qnh;
    if (fetchQNHfromOpenMeteo(qnh)) {
      seaLevelPressure_hPa_current = qnh;
      saveSeaLevelPressure(seaLevelPressure_hPa_current);
      displayCenteredStatus("QNH Updated", String(seaLevelPressure_hPa_current,2) + " hPa");
      delay(QNH_UPDATED_DISPLAY_MS);
    }
  }
#endif

  lastSensorReadMillis = millis();
  oledScreenStateChangeMillis = millis();
#if ENABLE_PERIODIC_QNH_REFRESH
  nextQNHFetchMillis = millis() + QNH_REFRESH_INTERVAL_MS;
#endif

  if (currentAppMode == MODE_OFFLINE) readBME680SensorData();
}

// ================== LOOP ==================
void loop() {
  bsecLoopTick(); // keep LP 3s serviced

  if (currentAppMode == MODE_OFFLINE) {
    if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
      readBME680SensorData();
      lastSensorReadMillis = millis();
    }
  } else {
    handleSensorAutoRetry();
  }

  updateOLEDDisplayContent();
  handleSerialInput();
  yield();
}

// ================== OLED ==================
void initOLED() {
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.clearDisplay();
  display.display();
}
void displayCenteredStatus(const String& l1,const String& l2){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  int16_t x1,y1; uint16_t w,h;
  display.getTextBounds(l1,0,0,&x1,&y1,&w,&h);
  display.setCursor((SCREEN_W - (int)w)/2, 16); display.print(l1);
  display.getTextBounds(l2,0,0,&x1,&y1,&w,&h);
  display.setCursor((SCREEN_W - (int)w)/2, 40); display.print(l2);
  display.display();
}
void displaySensorDataScreen1(){
  display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0,0);  snprintf(oledBuffer,sizeof(oledBuffer),"Temp  %.1f C", gTemp); display.print(oledBuffer);
  display.setCursor(0,16); snprintf(oledBuffer,sizeof(oledBuffer),"Hum   %.1f %%", gHum);  display.print(oledBuffer);
  display.setCursor(0,32); snprintf(oledBuffer,sizeof(oledBuffer),"Pres  %d hPa", (int)lroundf(gPress)); display.print(oledBuffer);
  display.setCursor(0,48); snprintf(oledBuffer,sizeof(oledBuffer),"Alt   %d m",   (int)lroundf(gAlt));   display.print(oledBuffer);
  display.display();
}
void displaySensorDataScreen2(){
  display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0,0);  snprintf(oledBuffer,sizeof(oledBuffer),"Gas  %.1f kOhm", gGasEMA_kOhm); display.print(oledBuffer);
  display.setCursor(0,16); snprintf(oledBuffer,sizeof(oledBuffer),"IAQs %.1f (%u)", gIAQstatic, gIAQacc); display.print(oledBuffer);
  display.setCursor(0,32); display.print("Air Quality:");
  display.setCursor(0,48); display.print(iaqCategory(gIAQstatic));
  display.display();
}
void updateOLEDDisplayContent(){
  if (currentAppMode == MODE_BME_ERROR) return;

  unsigned long now=millis();
  unsigned long elapsed=now - oledScreenStateChangeMillis;

  switch (currentOledScreenState){
    case OLED_STATE_DATA_SCREEN_1:
      if (elapsed >= OLED_DATA_SCREEN_1_DURATION_MS){
        currentOledScreenState = OLED_STATE_DATA_SCREEN_2;
        oledScreenStateChangeMillis = now;
      }
      break;

    case OLED_STATE_DATA_SCREEN_2:
      if (elapsed >= OLED_DATA_SCREEN_2_DURATION_MS){
        currentOledScreenState = OLED_STATE_OFF;
        oledScreenStateChangeMillis = now;

        display.ssd1306_command(SSD1306_DISPLAYOFF);

        bool didFetch=false;
#if ENABLE_PERIODIC_QNH_REFRESH
        if (millis() >= nextQNHFetchMillis) {
          unsigned long tStart = millis();
          float qnh;
          if (fetchQNHfromOpenMeteo(qnh)) {
            seaLevelPressure_hPa_current = qnh;
            saveSeaLevelPressure(seaLevelPressure_hPa_current);
#if SHOW_QNH_UPDATED_ON_PERIODIC
            display.ssd1306_command(SSD1306_DISPLAYON);
            display.clearDisplay(); display.display(); // ghost-clear
            displayCenteredStatus("QNH Updated", String(seaLevelPressure_hPa_current,2) + " hPa");
            delay(QNH_UPDATED_DISPLAY_MS);
            display.ssd1306_command(SSD1306_DISPLAYOFF);
#endif
          }
          nextQNHFetchMillis = millis() + QNH_REFRESH_INTERVAL_MS;
          unsigned long spent = millis() - tStart;
          if (spent < OLED_OFF_DURATION_MS) lightSleepDuringOff(OLED_OFF_DURATION_MS - spent);
          didFetch=true;
        }
#endif
        if (!didFetch) lightSleepDuringOff(OLED_OFF_DURATION_MS);
        return;
      }
      break;

    case OLED_STATE_OFF:
      if (elapsed >= OLED_OFF_DURATION_MS){
        currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
        oledScreenStateChangeMillis = now;
        display.ssd1306_command(SSD1306_DISPLAYON);
        display.clearDisplay(); display.display(); // ghost-clear
        delay(50);
      } else {
        return;
      }
      break;

    case OLED_STATE_ERROR_SCREEN:
      break;
  }

  if (currentOledScreenState == OLED_STATE_DATA_SCREEN_1){
    displaySensorDataScreen1();
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_2){
    displaySensorDataScreen2();
  } else if (currentOledScreenState == OLED_STATE_ERROR_SCREEN){
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
    display.setCursor(20,16); display.print("Sensor Error");
    display.setCursor(20,40); display.print("Check Conn.!");
    display.display();
  }
}

// ================== BSEC ==================
bool loadBsecState(){
  uint32_t magic=0; EEPROM.get(BSEC_STATE_VALID_ADDR, magic);
  if (magic != BSEC_STATE_VALID_MAGIC) return false;
  uint8_t blob[BSEC_STATE_MAXLEN];
  for (int i=0;i<BSEC_STATE_MAXLEN;i++) EEPROM.get(BSEC_STATE_ADDR+i, blob[i]);
  iaqSensor.setState(blob);
  return true;
}
void saveBsecState(){
  uint8_t blob[BSEC_STATE_MAXLEN]; memset(blob,0,sizeof(blob));
  iaqSensor.getState(blob);
  for (int i=0;i<BSEC_STATE_MAXLEN;i++) EEPROM.put(BSEC_STATE_ADDR+i, blob[i]);
  uint32_t magic=BSEC_STATE_VALID_MAGIC;
  EEPROM.put(BSEC_STATE_VALID_ADDR, magic);
  EEPROM.commit();
}
bool initBSEC(){
  iaqSensor.begin(BME_ADDRESS, Wire);
  if (iaqSensor.bsecStatus < BSEC_OK || iaqSensor.bme68xStatus != BME68X_OK) return false;

  bsec_virtual_sensor_t list[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_GAS
  };
  iaqSensor.updateSubscription(list, sizeof(list)/sizeof(list[0]), BSEC_SAMPLE_RATE_LP); // ~3 s
  if (iaqSensor.bsecStatus < BSEC_OK || iaqSensor.bme68xStatus != BME68X_OK) return false;

  loadBsecState();
  lastBsecSaveMs = millis();
  gIAQaccPrev = iaqSensor.iaqAccuracy;
  bsecActive = true; bsecHasData = false;
  return true;
}

// Poll BSEC each loop; copy outputs when ready
void bsecLoopTick(){
  if (!bsecActive) return;
  if (!iaqSensor.run()) return;

  gIAQ       = iaqSensor.iaq;
  gIAQstatic = iaqSensor.staticIaq;
  gIAQacc    = iaqSensor.iaqAccuracy;
  gTemp      = iaqSensor.temperature;
  gHum       = iaqSensor.humidity;
  gPress     = iaqSensor.pressure / 100.0f;
  gGas_kOhm  = iaqSensor.gasResistance / 1000.0f;

  float ratio = gPress / seaLevelPressure_hPa_current;
  gAlt = 44330.0f * (1.0f - powf(ratio, 0.1903f));

  bsecHasData = true;
  lastBsecDataMs = millis();

  // periodic save
  if (millis() - lastBsecSaveMs >= BSEC_SAVE_INTERVAL_MS && gIAQacc >= 3){
    saveBsecState(); lastBsecSaveMs = millis();
  }
  // event-based save when accuracy rises (>=2, esp. reach 3)
  if (gIAQacc > gIAQaccPrev && gIAQacc >= 2) {
    if (millis() - lastBsecSaveMs >= BSEC_MIN_SAVE_GAP_MS){
      saveBsecState(); lastBsecSaveMs = millis();
    }
  }
  gIAQaccPrev = gIAQacc;
}

// Use latest values, then EMA/baseline/hysteresis and wake OLED
void readBME680SensorData(){
  if (bsecActive){
    if (!bsecHasData || (millis() - lastBsecDataMs > 20000)){ // >20 s no data in LP
      currentAppMode = MODE_BME_ERROR;
      displayCenteredStatus("BSEC Read Fail!","Check Conn.!");
      scheduleSensorRetryInitial();
      return;
    }
  } else {
    if (!bme.performReading()){
      currentAppMode=MODE_BME_ERROR;
      displayCenteredStatus("BME680 Read Fail!","Check Conn.!");
      scheduleSensorRetryInitial();
      return;
    }
    gTemp=bme.temperature; gHum=bme.humidity;
    gPress=bme.pressure/100.0f; gAlt=bme.readAltitude(seaLevelPressure_hPa_current);
    gGas_kOhm=bme.gas_resistance/1000.0f;
  }

  if (isnan(gGasEMA_kOhm)) gGasEMA_kOhm = gGas_kOhm;
  else gGasEMA_kOhm = GAS_EMA_ALPHA*gGas_kOhm + (1.0f-GAS_EMA_ALPHA)*gGasEMA_kOhm;

  if (isnan(gasBaseline_kOhm)) gasBaseline_kOhm = gGasEMA_kOhm;
  float d = gGasEMA_kOhm - gasBaseline_kOhm;
  gasBaseline_kOhm += (d>0 ? BASELINE_ALPHA_UP : BASELINE_ALPHA_DOWN) * d;

  gasSampleCounter++;
  if (!gasBaselineReady && gasSampleCounter>=GAS_BASELINE_READY_SAMPLES){
    gasBaselineReady=true; saveGasBaseline(gasBaseline_kOhm, true);
  }

  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay(); display.display(); // ghost-clear
  delay(50);
}

// ================== Fallback BME680 ==================
void initBME680(){
  uint8_t attempts=0;
  while(!bme.begin(BME_ADDRESS)){
    attempts++;
    displayCenteredStatus("BME680 Init Fail!","Retry "+String(attempts));
    delay(1500);
    if (attempts>=15){
      currentAppMode=MODE_BME_ERROR;
      displayCenteredStatus("BME680 Fail!","Error Mode");
      scheduleSensorRetryInitial();
      return;
    }
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320,150);
}
bool selfTestBME680(){
  for (uint8_t i=0;i<3;i++){
    if (bme.performReading()){
      if (!isnan(bme.temperature) && !isnan(bme.humidity) && !isnan(bme.pressure)) return true;
    }
    delay(100);
  }
  return false;
}

// ================== Auto-retry (backoff) ==================
void scheduleSensorRetryInitial(){ sensorRetryBackoffMs = 1000; nextSensorRetryMillis = millis() + sensorRetryBackoffMs; }
void handleSensorAutoRetry(){
  if (millis() < nextSensorRetryMillis) return;

  if (initBSEC()){
    currentAppMode = MODE_OFFLINE;
    bsecActive = true; bsecHasData = false;
    displayCenteredStatus("Sensor Recovered","");
    delay(800);
    lastSensorReadMillis = millis() - SENSOR_READ_INTERVAL_MS;
    oledScreenStateChangeMillis = millis();
    display.ssd1306_command(SSD1306_DISPLAYON);
    display.clearDisplay(); display.display();
    return;
  }
  initBME680();
  if (selfTestBME680()){
    currentAppMode = MODE_OFFLINE;
    bsecActive = false;
    displayCenteredStatus("Sensor Recovered","");
    delay(800);
    lastSensorReadMillis = millis() - SENSOR_READ_INTERVAL_MS;
    oledScreenStateChangeMillis = millis();
    display.ssd1306_command(SSD1306_DISPLAYON);
    display.clearDisplay(); display.display();
    return;
  }
  if (sensorRetryBackoffMs < 60000) sensorRetryBackoffMs *= 2;
  if (sensorRetryBackoffMs > 60000) sensorRetryBackoffMs = 60000;
  nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
}

// ================== Persistence ==================
void loadPersistent(){
  uint32_t magic=0; EEPROM.get(EEPROM_MAGIC_ADDR, magic);
  if (magic != EEPROM_MAGIC_VALUE){
    seaLevelPressure_hPa_current = 1012.50f;
    gasBaseline_kOhm = NAN; gasBaselineReady=false;
    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.put(GAS_BASELINE_ADDR, gasBaseline_kOhm);
    EEPROM.put(GAS_BASELINE_READY_ADDR, (uint8_t)gasBaselineReady);
    EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_VALUE);
    EEPROM.commit();
    return;
  }
  EEPROM.get(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
  if (isnan(seaLevelPressure_hPa_current) || seaLevelPressure_hPa_current<870.0f || seaLevelPressure_hPa_current>1100.0f){
    seaLevelPressure_hPa_current = 1012.50f;
    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.commit();
  }
  EEPROM.get(GAS_BASELINE_ADDR, gasBaseline_kOhm);
  uint8_t rdy=0; EEPROM.get(GAS_BASELINE_READY_ADDR, rdy);
  gasBaselineReady = (rdy!=0);
}
void saveSeaLevelPressure(float p){ EEPROM.put(SEA_LEVEL_PRESSURE_ADDR,p); EEPROM.commit(); }
void saveGasBaseline(float b,bool rdy){ EEPROM.put(GAS_BASELINE_ADDR,b); EEPROM.put(GAS_BASELINE_READY_ADDR,(uint8_t)rdy); EEPROM.commit(); }

// ================== Serial (baseline ops) ==================
void handleSerialInput(){
  static String cmd="";
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n'){
      cmd.trim();
      if (cmd=="getbaseline"){
        Serial.print(F("Baseline(kOhm): "));
        Serial.println(isnan(gasBaseline_kOhm)?String("NaN"):String(gasBaseline_kOhm,2));
        Serial.print(F("BaselineReady: ")); Serial.println(gasBaselineReady?"true":"false");
      } else if (cmd=="resetbaseline"){
        gasBaseline_kOhm=NAN; gasBaselineReady=false; saveGasBaseline(gasBaseline_kOhm, false);
        Serial.println(F("Baseline reset."));
      }
      cmd="";
    } else { cmd+=c; }
  }
}

// ================== BSEC-aware light sleep ==================
void lightSleepDuringOff(unsigned long ms){
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();                 // radio off

  unsigned long end = millis() + ms;
  while ((long)(end - millis()) > 0) {   // signed compare
    unsigned long slice = 350;           // ~0.35 s (kejar LP 3 s)
    unsigned long remain = end - millis();
    if (slice > remain) slice = remain;

    delay(slice);                        // idle
    bsecLoopTick();                      // keep BSEC fed
    yield();
  }

  WiFi.forceSleepWake();                 // wake core (WiFi tetap OFF)
  delay(2);
}

// ================== Open-Meteo fetch ==================
static void omBuildPathCurrent(String& out){
  out.reserve(120);
  out  = "/v1/forecast?latitude="; out += String(OM_LAT, 5);
  out += "&longitude="; out += String(OM_LON, 5);
  out += "&current=pressure_msl";
}
static void omBuildPathHourly(String& out){
  out.reserve(140);
  out  = "/v1/forecast?latitude="; out += String(OM_LAT, 5);
  out += "&longitude="; out += String(OM_LON, 5);
  out += "&hourly=pressure_msl";
}
static bool extractPressureMSL_Current(const String& body, float& val){
  int cur = body.indexOf("\"current\"");
  int start = (cur >= 0) ? cur : 0;
  int k = body.indexOf("\"pressure_msl\"", start);
  if (k < 0) return false;
  int c = body.indexOf(':', k); if (c < 0) return false;
  int s = c + 1; while (s < (int)body.length() && (body[s]==' ')) s++;
  int e = s; while (e < (int)body.length() && (isdigit(body[e]) || body[e]=='.' || body[e]=='-')) e++;
  val = body.substring(s, e).toFloat();
  return true;
}
static bool extractPressureMSL_Hourly_Last(const String& body, float& val){
  int k = body.lastIndexOf("\"pressure_msl\"");
  if (k < 0) return false;
  int lb = body.indexOf('[', k);
  int rb = body.indexOf(']', lb);
  if (lb < 0 || rb < 0) return false;
  String arr = body.substring(lb+1, rb);
  int i = arr.length()-1; while (i>=0 && (arr[i]==' ' || arr[i]==',')) i--;;
  int end=i; while (i>=0 && (isdigit(arr[i]) || arr[i]=='.' || arr[i]=='-')) i--;
  if (end < 0) return false;
  val = arr.substring(i+1, end+1).toFloat();
  return true;
}
static bool httpGetOpenMeteo(const String& path, String& resp){
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) delay(50);
  if (WiFi.status() != WL_CONNECTED) { WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin(); return false; }

  WiFiClient client;
  if (!client.connect("api.open-meteo.com", 80)) { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin(); return false; }

  client.print(String("GET ") + path + " HTTP/1.0\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n");
  resp = ""; resp.reserve(768);
  unsigned long t1 = millis();
  while ((client.connected() || client.available()) && millis() - t1 < QNH_FETCH_TIMEOUT_MS) {
    while (client.available()) resp += (char)client.read();
    delay(1);
  }
  client.stop();
  WiFi.disconnect(true); WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin();
  int hdr = resp.indexOf("\r\n\r\n"); if (hdr >= 0) resp = resp.substring(hdr + 4);
  resp.trim();
  return resp.length() > 0;
}
static bool fetchQNHfromOpenMeteo(float &out_hPa){
  String resp, path;
  omBuildPathCurrent(path);
  if (httpGetOpenMeteo(path, resp)){
    float v; if (extractPressureMSL_Current(resp, v) && v>=870.0f && v<=1100.0f) { out_hPa=v; return true; }
  }
  omBuildPathHourly(path);
  if (httpGetOpenMeteo(path, resp)){
    float v; if (extractPressureMSL_Hourly_Last(resp, v) && v>=870.0f && v<=1100.0f) { out_hPa=v; return true; }
  }
  return false;
}
