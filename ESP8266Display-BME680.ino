#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SSD1306Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>

// --- I2C & address ---
#define OLED_ADDRESS 0x3C
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define BME_ADDRESS  0x76

// --- Timing ---
const unsigned long SENSOR_READ_INTERVAL_MS = 30000;
const unsigned long OLED_DATA_SCREEN_1_DURATION_MS = 7000;
const unsigned long OLED_DATA_SCREEN_2_DURATION_MS = 3000;
const unsigned long OLED_OFF_DURATION_MS =
  SENSOR_READ_INTERVAL_MS - (OLED_DATA_SCREEN_1_DURATION_MS + OLED_DATA_SCREEN_2_DURATION_MS);

// --- BME680 robustness ---
const uint8_t  BME680_RELIABLE_READ_RETRIES = 5;
const uint16_t BME680_RELIABLE_READ_RETRY_DELAY_MS = 50;
const uint16_t BME680_INIT_RETRY_INTERVAL_MS = 1500;
const uint8_t  BME680_MAX_INIT_ATTEMPTS = 15;

// --- EEPROM layout ---
#define EEPROM_SIZE 512
#define SEA_LEVEL_PRESSURE_ADDR 0
#define GAS_BASELINE_ADDR       4
#define GAS_BASELINE_READY_ADDR 8
#define EEPROM_MAGIC_ADDR       9
#define EEPROM_MAGIC_VALUE      0xB6680A11

// --- Wi-Fi + Open-Meteo ---
#define ENABLE_BOOT_QNH_WIFI          1
#define ENABLE_PERIODIC_QNH_REFRESH   1
#define SHOW_QNH_UPDATED_ON_PERIODIC  1
#define QNH_UPDATED_DISPLAY_MS        2000

const char* WIFI_SSID = "Kosan bu nata";
const char* WIFI_PASS = "immodium";

const float OM_LAT = -6.8981f;
const float OM_LON = 107.6349f;

const uint16_t WIFI_CONNECT_TIMEOUT_MS = 6000;
const uint16_t QNH_FETCH_TIMEOUT_MS    = 3000;
const unsigned long QNH_REFRESH_INTERVAL_MS = 3UL*60UL*60UL*1000UL; // 3h

// --- Devices ---
SSD1306Wire display(OLED_ADDRESS, I2C_SDA_GPIO, I2C_SCL_GPIO);
Adafruit_BME680 bme(&Wire);

// --- Globals ---
char   oledBuffer[64];
float gTemp=0, gHum=0, gPress=0, gAlt=0, gGas_kOhm=0, gGasEMA_kOhm=NAN;
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

// --- IAQ feel ---
const float GAS_EMA_ALPHA=0.20f;
const float BASELINE_ALPHA_UP=0.02f, BASELINE_ALPHA_DOWN=0.001f;
const float TH_VERY_POOR=0.60f, TH_POOR=0.75f, TH_NORMAL=0.90f, TH_GOOD=1.05f;
const float STATUS_HYST_KOHM=3.0f;
uint32_t gasSampleCounter=0;
const uint32_t GAS_BASELINE_READY_SAMPLES=2880;

enum GasStatus { VS_VERY_POOR, VS_POOR, VS_NORMAL, VS_GOOD, VS_EXCELLENT };
GasStatus lastStatus = VS_NORMAL;

// --- Fwd decl ---
void initOLED(); void initBME680(); bool selfTestBME680();
void readBME680SensorData();
bool readBME680SensorReliably(float &t,float &h,float &p_Pa,float &alt,float &gas_k);
void updateOLEDDisplayContent();
void displayCenteredStatus(const String&, const String&);
void displaySensorDataScreen1(); void displaySensorDataScreen2();
String getGasStatus(float gas_kOhm);
String getGasStatusWithHysteresis(float gas_kOhm);
void loadPersistent(); void saveSeaLevelPressure(float p);
void saveGasBaseline(float b,bool rdy);
void handleSerialInput();
void lightSleepDuringOff(unsigned long ms);

// Open-Meteo helpers
static void omBuildPath(String& out);
static bool extractPressureMSL_Current(const String& body, float& val);
static bool fetchQNHfromOpenMeteo(float &out_hPa);

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200); // kept for manual commands
  EEPROM.begin(EEPROM_SIZE);
  loadPersistent();

  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
  Wire.setClock(400000);

  initOLED();
  displayCenteredStatus("BME680 Monitor","Initializing...");
  delay(500);

  initBME680();
  if (!selfTestBME680()) {
    Wire.setClock(100000);
    initBME680();
    if (!selfTestBME680()) {
      currentAppMode = MODE_BME_ERROR;
      displayCenteredStatus("BME680 Fail!","Error Mode");
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

// -------------------- Loop --------------------
void loop() {
  if (currentAppMode == MODE_OFFLINE) {
    if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
      readBME680SensorData();
      lastSensorReadMillis = millis();
    }
  }
  updateOLEDDisplayContent();
  handleSerialInput();
  yield();
}

// -------------------- OLED --------------------
void initOLED() {
  if (!display.init()) { while(1){ yield(); } }
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
}

void displayCenteredStatus(const String& l1,const String& l2){
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64,15,l1);
  display.drawString(64,40,l2);
  display.display();
}

void displaySensorDataScreen1(){
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  snprintf(oledBuffer,sizeof(oledBuffer),"Temp: %.2f C", gTemp);  display.drawString(0,0,oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"Hum: %.2f %%", gHum);   display.drawString(0,16,oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"Press: %.1f hPa", gPress); display.drawString(0,32,oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"Alti: %.2f m", gAlt);  display.drawString(0,48,oledBuffer);
}

void displaySensorDataScreen2(){
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  snprintf(oledBuffer,sizeof(oledBuffer),"Gas: %.2f kOhm", gGasEMA_kOhm); display.drawString(0,0,oledBuffer);
  display.drawString(0,16,"Status:");
  display.drawString(0,32,getGasStatusWithHysteresis(gGasEMA_kOhm));
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
        display.displayOff();

        bool didFetch=false;
#if ENABLE_PERIODIC_QNH_REFRESH
        if (millis() >= nextQNHFetchMillis) {
          unsigned long tStart = millis();
          float qnh;
          if (fetchQNHfromOpenMeteo(qnh)) {
            seaLevelPressure_hPa_current = qnh;
            saveSeaLevelPressure(seaLevelPressure_hPa_current);
#if SHOW_QNH_UPDATED_ON_PERIODIC
            display.displayOn();
            displayCenteredStatus("QNH Updated", String(seaLevelPressure_hPa_current,2) + " hPa");
            delay(QNH_UPDATED_DISPLAY_MS);
            display.displayOff();
#endif
          }
          nextQNHFetchMillis = millis() + QNH_REFRESH_INTERVAL_MS;
          unsigned long spent = millis() - tStart;
          if (spent < OLED_OFF_DURATION_MS) {
            lightSleepDuringOff(OLED_OFF_DURATION_MS - spent);
          }
          didFetch=true;
        }
#endif
        if (!didFetch) {
          lightSleepDuringOff(OLED_OFF_DURATION_MS);
        }
        return; // stay off
      }
      break;

    case OLED_STATE_OFF:
      if (elapsed >= OLED_OFF_DURATION_MS){
        currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
        oledScreenStateChangeMillis = now;
        display.displayOn();
        delay(50);
      } else {
        return; // keep off
      }
      break;

    case OLED_STATE_ERROR_SCREEN:
      break;
  }

  display.clear();
  if (currentOledScreenState == OLED_STATE_DATA_SCREEN_1){
    displaySensorDataScreen1(); display.display();
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_2){
    displaySensorDataScreen2(); display.display();
  } else if (currentOledScreenState == OLED_STATE_ERROR_SCREEN){
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64,15,"Sensor Error");
    display.drawString(64,40,"Check Conn.!");
    display.display();
  }
}

// -------------------- BME680 --------------------
void initBME680(){
  uint8_t attempts=0;
  while(!bme.begin(BME_ADDRESS)){
    attempts++;
    displayCenteredStatus("BME680 Init Fail!","Retry "+String(attempts));
    delay(BME680_INIT_RETRY_INTERVAL_MS);
    if (attempts>=BME680_MAX_INIT_ATTEMPTS){
      currentAppMode=MODE_BME_ERROR;
      displayCenteredStatus("BME680 Fail!","Error Mode");
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

void readBME680SensorData(){
  float t,h,p_Pa,alt,gas_k;
  if (!readBME680SensorReliably(t,h,p_Pa,alt,gas_k)){
    currentAppMode=MODE_BME_ERROR;
    displayCenteredStatus("Sensor Read Fail!","Check Conn.!");
    return;
  }
  gTemp=t; gHum=h; gPress=p_Pa/100.0f; gAlt=alt; gGas_kOhm=gas_k;

  if (isnan(gGasEMA_kOhm)) gGasEMA_kOhm = gGas_kOhm;
  else gGasEMA_kOhm = GAS_EMA_ALPHA*gGas_kOhm + (1.0f-GAS_EMA_ALPHA)*gGasEMA_kOhm;

  if (isnan(gasBaseline_kOhm)) gasBaseline_kOhm = gGasEMA_kOhm;
  float d = gGasEMA_kOhm - gasBaseline_kOhm;
  if (d>0) gasBaseline_kOhm += BASELINE_ALPHA_UP*d; else gasBaseline_kOhm += BASELINE_ALPHA_DOWN*d;

  gasSampleCounter++;
  if (!gasBaselineReady && gasSampleCounter>=GAS_BASELINE_READY_SAMPLES){
    gasBaselineReady=true; saveGasBaseline(gasBaseline_kOhm, true);
  }

  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
  display.displayOn(); delay(50);
}

bool readBME680SensorReliably(float &temp,float &hum,float &press_Pa,float &alt,float &gas_kOhm){
  for (uint8_t i=0;i<BME680_RELIABLE_READ_RETRIES;i++){
    if (bme.performReading()){
      temp=bme.temperature; hum=bme.humidity; press_Pa=bme.pressure;
      alt=bme.readAltitude(seaLevelPressure_hPa_current);
      gas_kOhm=bme.gas_resistance/1000.0f;
      if (!isnan(temp)&&!isnan(hum)&&!isnan(press_Pa)&&!isnan(alt)&&!isnan(gas_kOhm) &&
          temp>=-40 && temp<=85 && hum>=0 && hum<=100 &&
          press_Pa>=30000 && press_Pa<=110000 && gas_kOhm>=0) return true;
    }
    delay(BME680_RELIABLE_READ_RETRY_DELAY_MS);
  }
  return false;
}

// -------------------- IAQ label --------------------
String getGasStatusWithHysteresis(float gas_k){
  if (isnan(gasBaseline_kOhm) || gasBaseline_kOhm<1.0f) return "Calibrating";
  float r = gas_k / gasBaseline_kOhm;
  GasStatus next = lastStatus;

  switch(lastStatus){
    case VS_VERY_POOR:
      if (r>=TH_POOR || gas_k>= (TH_POOR*gasBaseline_kOhm + STATUS_HYST_KOHM)) next=VS_POOR;
      break;
    case VS_POOR:
      if (r<TH_VERY_POOR && gas_k< (TH_VERY_POOR*gasBaseline_kOhm - STATUS_HYST_KOHM)) next=VS_VERY_POOR;
      else if (r>=TH_NORMAL || gas_k>= (TH_NORMAL*gasBaseline_kOhm + STATUS_HYST_KOHM)) next=VS_NORMAL;
      break;
    case VS_NORMAL:
      if (r<TH_POOR && gas_k< (TH_POOR*gasBaseline_kOhm - STATUS_HYST_KOHM)) next=VS_POOR;
      else if (r>=TH_GOOD || gas_k>= (TH_GOOD*gasBaseline_kOhm + STATUS_HYST_KOHM)) next=VS_GOOD;
      break;
    case VS_GOOD:
      if (r<TH_NORMAL && gas_k< (TH_NORMAL*gasBaseline_kOhm - STATUS_HYST_KOHM)) next=VS_NORMAL;
      else if (r>=1.10f || gas_k>= (1.10f*gasBaseline_kOhm + STATUS_HYST_KOHM)) next=VS_EXCELLENT;
      break;
    case VS_EXCELLENT:
      if (r<TH_GOOD && gas_k< (TH_GOOD*gasBaseline_kOhm - STATUS_HYST_KOHM)) next=VS_GOOD;
      break;
  }
  lastStatus = next;
  return getGasStatus(gas_k);
}

String getGasStatus(float gas_k){
  if (isnan(gasBaseline_kOhm) || gasBaseline_kOhm<1.0f) return "Calibrating";
  float r = gas_k / gasBaseline_kOhm;
  if (r <= TH_VERY_POOR) return "Very Poor";
  else if (r <= TH_POOR) return "Poor";
  else if (r <= TH_NORMAL) return "Normal";
  else if (r <= TH_GOOD) return "Good";
  else return "Excellent";
}

// -------------------- Persistence --------------------
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

// -------------------- Serial --------------------
void handleSerialInput(){
  static String cmd="";
  while (Serial.available()){
    char c=Serial.read();
    if (c=='\n'){
      cmd.trim();
      if (cmd=="getbaseline"){
        Serial.print("Baseline(kOhm): ");
        Serial.println(isnan(gasBaseline_kOhm)?String("NaN"):String(gasBaseline_kOhm,2));
        Serial.print("BaselineReady: "); Serial.println(gasBaselineReady?"true":"false");
      } else if (cmd=="resetbaseline"){
        gasBaseline_kOhm=NAN; gasBaselineReady=false; saveGasBaseline(gasBaseline_kOhm, false);
        Serial.println("Baseline reset.");
      }
      cmd="";
    } else { cmd+=c; }
  }
}

// -------------------- Light sleep --------------------
void lightSleepDuringOff(unsigned long ms){
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(ms);
  WiFi.forceSleepWake();
  delay(1);
}

// -------------------- Open-Meteo fetch --------------------
static void omBuildPath(String& out){
  out.reserve(120);
  out  = "/v1/forecast?latitude="; out += String(OM_LAT, 5);
  out += "&longitude="; out += String(OM_LON, 5);
  out += "&current=pressure_msl";
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

static bool fetchQNHfromOpenMeteo(float &out_hPa){
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) delay(50);
  if (WiFi.status() != WL_CONNECTED) { 
    WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin(); 
    return false; 
  }

  WiFiClient client;
  if (!client.connect("api.open-meteo.com", 80)) {
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF); WiFi.forceSleepBegin(); 
    return false;
  }

  String path; omBuildPath(path);
  client.print(String("GET ") + path + " HTTP/1.0\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n");

  String resp; resp.reserve(768);
  unsigned long t1 = millis();
  while ((client.connected() || client.available()) && millis() - t1 < QNH_FETCH_TIMEOUT_MS) {
    while (client.available()) resp += (char)client.read();
    delay(1);
  }
  client.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();

  int hdr = resp.indexOf("\r\n\r\n");
  String body = (hdr >= 0) ? resp.substring(hdr + 4) : resp;
  body.trim();

  float v;
  if (extractPressureMSL_Current(body, v)) {
    if (v >= 870.0f && v <= 1100.0f) { out_hPa = v; return true; }
  }
  return false;
}
