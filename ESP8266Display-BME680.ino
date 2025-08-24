#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <bsec.h>
#include <math.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>

// ================== Hardware & Display ==================
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define OLED_ADDRESS 0x3C
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_RESET -1
#define BME_ADDRESS 0x76

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// Low-level SSD1306 helpers (karena Adafruit lib tidak expose setContrast())
static inline void oledCmd(uint8_t c){ display.ssd1306_command(c); }
static inline void oledSetContrast(uint8_t v){ oledCmd(SSD1306_SETCONTRAST); oledCmd(v); }

// ================== Timing ==================
const unsigned long SENSOR_READ_INTERVAL_MS = 30000;  // normal refresh
const unsigned long OLED_DATA_SCREEN_1_DURATION_MS = 5000;
const unsigned long OLED_DATA_SCREEN_2_DURATION_MS = 5000;

// ================== EEPROM Layout ==================
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

// ================== QNH ==================
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
const unsigned long QNH_REFRESH_INTERVAL_MS = 1UL*60UL*60UL*1000UL;

// ================== BSEC / Sensor ==================
Bsec iaqSensor;
char  oledBuffer[64];
float gTemp=0, gHum=0, gPress=0, gAlt=0, gGas_kOhm=0, gGasEMA_kOhm=NAN;
float gIAQ=NAN, gIAQstatic=NAN, gIAQstaticDisp=NAN;
uint8_t gIAQacc=0, gIAQaccPrev=0, gIAQaccDisp=0;

float seaLevelPressure_hPa_current = 1012.50f;
float gasBaseline_kOhm = NAN; bool gasBaselineReady=false;

enum AppMode { MODE_OFFLINE, MODE_BME_ERROR };
AppMode currentAppMode = MODE_OFFLINE;

enum OledDisplayState { OLED_STATE_ERROR_SCREEN, OLED_STATE_DATA_SCREEN_1, OLED_STATE_DATA_SCREEN_2 };
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;

unsigned long lastSensorReadMillis=0, oledScreenStateChangeMillis=0;
#if ENABLE_PERIODIC_QNH_REFRESH
unsigned long nextQNHFetchMillis=0;
#endif

const float IAQ_VAR_ALPHA = 0.10f;
float iaqMeanEWMA = NAN, iaqVarEWMA = 0.0f;

const float GAS_EMA_ALPHA=0.20f;
const float BASELINE_ALPHA_UP=0.02f, BASELINE_ALPHA_DOWN=0.001f;
uint32_t gasSampleCounter=0;
const uint32_t GAS_BASELINE_READY_SAMPLES=2880;

const uint8_t  PORT_DECIM_N   = 10;
const uint8_t  PORT_BUF       = 24;
const float    PORT_TH_DP     = 3.5f;
const float    PORT_TH_DH     = 8.0f;
const float    PORT_TH_DIAQ   = 25.0f;
const uint8_t  PORT_RESUME_OK = 6;
float   portBufP[PORT_BUF], portBufH[PORT_BUF], portBufI[PORT_BUF];
uint8_t portFill=0, portW=0, portDecim=0, portStableCount=0;
bool    baselineFrozen=false;

const float ALT_EMA_ALPHA = 0.25f;
const float ALT_OUTLIER_M = 18.0f;
float gAltSmooth = NAN, altRaw3[3]={NAN,NAN,NAN};
uint8_t altIdx=0, altCnt=0;

unsigned long nextSensorRetryMillis = 0;
unsigned long sensorRetryBackoffMs  = 1000;
unsigned long lastBsecSaveMs = 0;
const unsigned long BSEC_SAVE_INTERVAL_MS = 4UL*60UL*60UL*1000UL;
const unsigned long BSEC_MIN_SAVE_GAP_MS  = 10UL*60UL*1000UL;

bool bsecActive = false;
bool bsecHasData = false;
unsigned long lastBsecDataMs = 0;

unsigned long bootMs = 0;
const unsigned long BOOT_GRACE_MS = 60000;
const unsigned long NO_DATA_TIMEOUT_BOOT_MS = 45000;
const unsigned long NO_DATA_TIMEOUT_RUN_MS  = 20000;

// --- incremental redraw ---
float prev_T=NAN, prev_H=NAN, prev_P=NAN, prev_Alt=NAN;
float prev_G=NAN, prev_IAQ=NAN;
uint8_t prev_Acc=255;
String prev_AQS="";
OledDisplayState lastDrawnState = OLED_STATE_ERROR_SCREEN;
const float TH_T=0.10f, TH_H=0.10f, TH_P=0.10f, TH_ALT=1.0f, TH_G=0.10f, TH_IAQ=0.50f;

// ================== Thermal Protection ==================
enum ThermalState { THERM_NORMAL, THERM_HOT_HOLD };
ThermalState thermal = THERM_NORMAL;
const float HOT_ENTER_C = 45.0f;      // masuk overheat
const float HOT_EXIT_C  = 41.0f;      // histeresis keluar
const uint8_t CONTRAST_NORMAL   = 0x7F; // kontras standar normal
const uint8_t CONTRAST_OVERHEAT = 0x04; // kontras minimal saat overheat

unsigned long nextSafetyProcessMs = 0;     // UI refresh 60 dtk di HOT_HOLD
const unsigned long SAFETY_PERIOD_MS = 60000;

// Posisi bergilir untuk anti burn-in pada tulisan Overheat
uint8_t overheatPosIndex = 0;

// ================== Utils ==================
static String iaqCategory(float x){
  if (isnan(x)) return "n/a";
  if (x <=  50) return "Excellent";
  if (x <= 100) return "Good";
  if (x <= 150) return "Light";
  if (x <= 200) return "Moderate";
  if (x <= 300) return "Unhealthy";
  return "Hazardous";
}
static float altitudeScientific(float press_hPa, float qnh_hPa, float t_C, float rh_pct){
  if (!(press_hPa>0 && qnh_hPa>0)) return NAN;
  float T = t_C + 273.15f;
  float es = 6.112f * expf((17.62f*t_C)/(243.12f + t_C));
  float e  = (rh_pct<=0?0: (rh_pct>=100? es : (rh_pct/100.0f)*es));
  if (e >= press_hPa*0.99f) e = press_hPa*0.99f;
  float w = 0.622f * e / (press_hPa - e);
  float q = w / (1.0f + w);
  float Tv = T * (1.0f + 0.61f*q);
  const float Rd = 287.05f, g0 = 9.80665f;
  float lnratio = logf(qnh_hPa/press_hPa);
  return (Rd * Tv / g0) * lnratio;
}
static inline float med3(float a,float b,float c){
  if (a>b) { float t=a;a=b;b=t; }
  if (b>c) { float t=b;b=c;c=t; }
  if (a>b) { float t=a;a=b;b=t; }
  return b;
}

// ================== OLED Routines ==================
void initOLED(){
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  oledCmd(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.setTextWrap(false);
  display.display();
  // tetapkan kontras normal sekali di awal
  oledSetContrast(CONTRAST_NORMAL);
}
void displayInitTwoLines(){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  int16_t bx,by; uint16_t bw,bh;
  display.setFont(&FreeSans12pt7b);
  display.getTextBounds("BME680", 0, 0, &bx,&by,&bw,&bh);
  display.setCursor((SCREEN_W - bw)/2, 22);
  display.print("BME680");
  display.setFont(&FreeSans9pt7b);
  display.getTextBounds("Initializing...", 0, 0, &bx,&by,&bw,&bh);
  display.setCursor((SCREEN_W - bw)/2, 48);
  display.print("Initializing...");
  display.display();
}
void displayCenteredStatus(const String& l1,const String& l2){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  int16_t bx,by; uint16_t bw,bh;
  display.setFont(&FreeSans12pt7b);
  display.getTextBounds(l1, 0, 0, &bx,&by,&bw,&bh);
  display.setCursor((SCREEN_W - bw)/2, 18); display.print(l1);
  display.getTextBounds(l2, 0, 0, &bx,&by,&bw,&bh);
  display.setCursor((SCREEN_W - bw)/2, 48); display.print(l2);
  display.display();
}

// Gambar ikon termometer sederhana di kiri teks
void drawThermometerIcon(int16_t x, int16_t y){
  // bulb
  display.fillCircle(x+6, y+18, 6, SSD1306_WHITE);
  // stem
  display.fillRect(x+5, y, 3, 18, SSD1306_WHITE);
}

// Posisi overheat yang berubah-ubah (5 posisi)
void getOverheatPos(uint8_t idx, int16_t &x, int16_t &y){
  switch(idx % 5){
    case 0: x=10; y=10; break;
    case 1: x=32; y=6;  break;
    case 2: x=20; y=28; break;
    case 3: x=48; y=18; break;
    default:x=8;  y=36; break;
  }
}

void displayOverheat(){
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&FreeSans12pt7b);

  int16_t x,y; getOverheatPos(overheatPosIndex, x, y);
  drawThermometerIcon(x, y);

  int16_t bx,by; uint16_t bw,bh;
  String t1="Overheat";
  display.getTextBounds(t1, 0, 0, &bx,&by,&bw,&bh);
  // Letakkan teks di samping ikon
  int16_t tx = x + 18;
  int16_t ty = y + 22; // baseline kasar untuk font 12pt
  if (tx + (int)bw > SCREEN_W) tx = SCREEN_W - bw;
  if (ty < 18) ty = 18;
  if (ty > 56) ty = 56;

  display.setCursor(tx, ty);
  display.print(t1);
  display.display();

  // siapkan posisi berikutnya untuk siklus anti burn-in
  overheatPosIndex = (overheatPosIndex + 1) % 5;
}

bool shouldRedrawScreen1(){
  if (currentOledScreenState != lastDrawnState) return true;
  if (isnan(prev_T) || isnan(prev_H) || isnan(prev_P) || isnan(prev_Alt)) return true;
  if (fabsf(gTemp - prev_T) >= TH_T) return true;
  if (fabsf(gHum - prev_H) >= TH_H) return true;
  if (fabsf(gPress - prev_P) >= TH_P) return true;
  if (fabsf(gAlt - prev_Alt) >= TH_ALT) return true;
  return false;
}
bool shouldRedrawScreen2(){
  if (currentOledScreenState != lastDrawnState) return true;
  String aqs = iaqCategory(gIAQstaticDisp);
  if (isnan(prev_G) || isnan(prev_IAQ)) return true;
  if (fabsf(gGasEMA_kOhm - prev_G) >= TH_G) return true;
  if (fabsf(gIAQstaticDisp - prev_IAQ) >= TH_IAQ) return true;
  if (gIAQaccDisp != prev_Acc) return true;
  if (aqs != prev_AQS) return true;
  return false;
}
void stampScreen1(){ prev_T=gTemp; prev_H=gHum; prev_P=gPress; prev_Alt=gAlt; lastDrawnState=OLED_STATE_DATA_SCREEN_1; }
void stampScreen2(){ prev_G=gGasEMA_kOhm; prev_IAQ=gIAQstaticDisp; prev_Acc=gIAQaccDisp; prev_AQS=iaqCategory(gIAQstaticDisp); lastDrawnState=OLED_STATE_DATA_SCREEN_2; }

void displaySensorDataScreen1(){
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);
  snprintf(oledBuffer,sizeof(oledBuffer),"T: %.2f C", gTemp);
  display.setCursor(0, 14); display.print(oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"H: %.2f %%", gHum);
  display.setCursor(0, 30); display.print(oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"P: %.1f hPa", gPress);
  display.setCursor(0, 46); display.print(oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"Alt: %d mdpl", (int)lroundf(gAlt));
  display.setCursor(0, 62); display.print(oledBuffer);
  display.display();
}
void displaySensorDataScreen2(){
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);
  snprintf(oledBuffer,sizeof(oledBuffer),"G: %.1f kOhm", gGasEMA_kOhm);
  display.setCursor(0, 14); display.print(oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"IAQ: %.1f", gIAQstaticDisp);
  display.setCursor(0, 30); display.print(oledBuffer);
  snprintf(oledBuffer,sizeof(oledBuffer),"Acc: %u", gIAQaccDisp);
  display.setCursor(0, 46); display.print(oledBuffer);
  display.setCursor(0, 62); display.print(String("AQS: ") + iaqCategory(gIAQstaticDisp));
  display.display();
}

// ================== Persistence ==================
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

// ================== BSEC init/run ==================
bool initBSEC(){
  Wire.beginTransmission(BME_ADDRESS); Wire.write(0xE0); Wire.write(0xB6); Wire.endTransmission();
  delay(10);
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
  iaqSensor.updateSubscription(list, sizeof(list)/sizeof(list[0]), BSEC_SAMPLE_RATE_LP); // 3s
  if (iaqSensor.bsecStatus < BSEC_OK || iaqSensor.bme68xStatus != BME68X_OK) return false;
  loadBsecState();
  lastBsecSaveMs = millis();
  gIAQaccPrev = iaqSensor.iaqAccuracy;
  gIAQaccDisp = gIAQaccPrev;
  bsecActive = true; bsecHasData = false;
  return true;
}
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

  float alt_tv = altitudeScientific(gPress, seaLevelPressure_hPa_current, gTemp, gHum);
  float alt_raw = isnan(alt_tv) ? (44330.0f * (1.0f - powf(gPress/seaLevelPressure_hPa_current, 0.1903f))) : alt_tv;
  altRaw3[altIdx] = alt_raw; altIdx=(altIdx+1)%3; if (altCnt<3) altCnt++;
  float med = (altCnt>=3)? med3(altRaw3[0],altRaw3[1],altRaw3[2]) : alt_raw;
  if (fabsf(alt_raw - med) > ALT_OUTLIER_M) alt_raw = med;
  if (isnan(gAltSmooth)) gAltSmooth=alt_raw; else gAltSmooth = ALT_EMA_ALPHA*alt_raw + (1.0f-ALT_EMA_ALPHA)*gAltSmooth;
  gAlt = gAltSmooth;

  bsecHasData = true;
  lastBsecDataMs = millis();

  if (isnan(gGasEMA_kOhm)) gGasEMA_kOhm = gGas_kOhm;
  else gGasEMA_kOhm = GAS_EMA_ALPHA*gGas_kOhm + (1.0f-GAS_EMA_ALPHA)*gGasEMA_kOhm;

  if (isnan(iaqMeanEWMA)) { iaqMeanEWMA = gIAQstatic; iaqVarEWMA = 0.0f; }
  float d = gIAQstatic - iaqMeanEWMA;
  iaqMeanEWMA += IAQ_VAR_ALPHA * d;
  iaqVarEWMA   = (1.0f-IAQ_VAR_ALPHA)*(iaqVarEWMA + IAQ_VAR_ALPHA*d*d);
  float vol = sqrtf(fmaxf(iaqVarEWMA, 0.0f));
  float aVar;
  if (vol <= 2.0f)      aVar = 0.10f;
  else if (vol >= 25.0f) aVar = 0.45f;
  else                   aVar = 0.10f + (vol-2.0f)*(0.35f/(25.0f-2.0f));
  if (gIAQacc >= 3) aVar *= 0.6f;
  if (gIAQacc <= 1) aVar = fmaxf(aVar, 0.28f);
  aVar = fminf(fmaxf(aVar, 0.08f), 0.45f);
  if (isnan(gIAQstaticDisp)) gIAQstaticDisp = gIAQstatic;
  else gIAQstaticDisp = aVar*gIAQstatic + (1.0f - aVar)*gIAQstaticDisp;

  if (millis() - lastBsecSaveMs >= BSEC_SAVE_INTERVAL_MS && gIAQacc >= 3){
    saveBsecState(); lastBsecSaveMs = millis();
  }
  if (gIAQacc > gIAQaccPrev && gIAQacc >= 2){
    if (millis() - lastBsecSaveMs >= BSEC_MIN_SAVE_GAP_MS){
      saveBsecState(); lastBsecSaveMs = millis();
    }
  }
  gIAQaccDisp = (gIAQacc >= 3) ? 3 : (gIAQacc==2 && gIAQaccDisp==3 ? 2 : gIAQacc);
  gIAQaccPrev = gIAQacc;

  // transport-aware baseline freeze
  if (++portDecim >= PORT_DECIM_N){
    portDecim=0;
    portBufP[portW]=gPress; portBufH[portW]=gHum; portBufI[portW]=gIAQstatic;
    portW=(portW+1)%PORT_BUF; if (portFill<PORT_BUF) portFill++;
    if (portFill>=2){
      int newest = (portW + PORT_BUF - 1) % PORT_BUF;
      int oldest = (portFill==PORT_BUF) ? portW : 0;
      float dP = portBufP[newest]-portBufP[oldest];
      float dH = portBufH[newest]-portBufH[oldest];
      float dI = portBufI[newest]-portBufI[oldest];
      bool moving = (fabsf(dP)>PORT_TH_DP) || (fabsf(dH)>PORT_TH_DH) || (fabsf(dI)>PORT_TH_DIAQ);
      if (moving){ baselineFrozen=true; portStableCount=0; }
      else { if (baselineFrozen){ if (++portStableCount>=PORT_RESUME_OK) baselineFrozen=false; } }
    }
  }
}

// ================== Thermal State Machine ==================
void enterHotHold(){
  thermal = THERM_HOT_HOLD;
  // hentikan WiFi (tidak fetch QNH saat overheat)
  WiFi.mode(WIFI_OFF);
  // kontras rendah & render overheat
  oledSetContrast(CONTRAST_OVERHEAT);
  displayOverheat();
  nextSafetyProcessMs = millis(); // proses pertama segera
}
void exitHotHold(){
  thermal = THERM_NORMAL;
  // pulihkan kontras standar
  oledSetContrast(CONTRAST_NORMAL);
  // paksa redraw
  lastDrawnState = OLED_STATE_ERROR_SCREEN;
  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
}

// ================== App Logic ==================
void readBME680SensorData(){
  if (!bsecActive) return;
  unsigned long noDataTimeout = (millis() - bootMs < BOOT_GRACE_MS) ? NO_DATA_TIMEOUT_BOOT_MS : NO_DATA_TIMEOUT_RUN_MS;
  if (!bsecHasData || (millis() - lastBsecDataMs > noDataTimeout)){
    currentAppMode = MODE_BME_ERROR;
    displayCenteredStatus("BSEC Read Fail!","Check Conn.!");
    // schedule retry
    sensorRetryBackoffMs = 1000;
    nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
    return;
  }
  // baseline adapt (if not frozen)
  if (!baselineFrozen){
    if (isnan(gasBaseline_kOhm)) gasBaseline_kOhm = gGasEMA_kOhm;
    float dv = gGasEMA_kOhm - gasBaseline_kOhm;
    gasBaseline_kOhm += (dv>0 ? BASELINE_ALPHA_UP : BASELINE_ALPHA_DOWN) * dv;
  }
  gasSampleCounter++;
  if (!gasBaselineReady && gasSampleCounter>=GAS_BASELINE_READY_SAMPLES){
    gasBaselineReady=true; saveGasBaseline(gasBaseline_kOhm, true);
  }

  // Thermal transitions
  if (thermal == THERM_NORMAL && gTemp >= HOT_ENTER_C){
    enterHotHold();
    return;
  }
  if (thermal == THERM_HOT_HOLD && gTemp <= HOT_EXIT_C){
    exitHotHold();
  }

  // Draw (NORMAL only)
  if (thermal == THERM_NORMAL){
    oledScreenStateChangeMillis = millis();
    // pastikan display ON (selalu ON) + kontras normal (sudah di-setup)
    display.clearDisplay(); display.display();
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
    // Render first screen immediately
    currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
    stampScreen1();
    displaySensorDataScreen1();
  } else {
    // THERM_HOT_HOLD -> keep Overheat screen
    displayOverheat();
  }
}

void updateOLEDDisplayContent(){
  if (currentAppMode == MODE_BME_ERROR) return;

  // Periodic QNH only in NORMAL
#if ENABLE_PERIODIC_QNH_REFRESH
  if (thermal == THERM_NORMAL && millis() >= nextQNHFetchMillis){
    float qnh;
    if (fetchQNHfromOpenMeteo(qnh)){
      seaLevelPressure_hPa_current = qnh;
      saveSeaLevelPressure(seaLevelPressure_hPa_current);
#if SHOW_QNH_UPDATED_ON_PERIODIC
      display.clearDisplay(); display.display();
      displayCenteredStatus("QNH Updated", String(seaLevelPressure_hPa_current,2) + " hPa");
      delay(QNH_UPDATED_DISPLAY_MS);
#endif
    }
    nextQNHFetchMillis = millis() + QNH_REFRESH_INTERVAL_MS;
  }
#endif

  if (thermal == THERM_HOT_HOLD){
    if (millis() >= nextSafetyProcessMs){
      displayOverheat();
      nextSafetyProcessMs = millis() + SAFETY_PERIOD_MS;
    }
    return;
  }

  // NORMAL display rotation 5s/5s
  unsigned long now=millis();
  unsigned long elapsed=now - oledScreenStateChangeMillis;

  if (currentOledScreenState == OLED_STATE_DATA_SCREEN_1){
    if (elapsed >= OLED_DATA_SCREEN_1_DURATION_MS){
      currentOledScreenState = OLED_STATE_DATA_SCREEN_2;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN;
    }
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_2){
    if (elapsed >= OLED_DATA_SCREEN_2_DURATION_MS){
      currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN;
    }
  }

  if (currentOledScreenState == OLED_STATE_DATA_SCREEN_1){
    if (shouldRedrawScreen1()){ displaySensorDataScreen1(); stampScreen1(); }
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_2){
    if (shouldRedrawScreen2()){ displaySensorDataScreen2(); stampScreen2(); }
  }
}

void scheduleSensorRetryInitial(){ sensorRetryBackoffMs = 1000; nextSensorRetryMillis = millis() + sensorRetryBackoffMs; }
void handleSensorAutoRetry(){
  if (millis() < nextSensorRetryMillis) return;
  if (initBSEC()){
    currentAppMode = MODE_OFFLINE;
    bsecActive = true; bsecHasData = false;
    displayCenteredStatus("Sensor Recovered","");
    delay(500);
    lastSensorReadMillis = millis() - SENSOR_READ_INTERVAL_MS;
    oledScreenStateChangeMillis = millis();
    display.clearDisplay(); display.display();
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
    return;
  }
  if (sensorRetryBackoffMs < 60000) sensorRetryBackoffMs *= 2;
  if (sensorRetryBackoffMs > 60000) sensorRetryBackoffMs = 60000;
  nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
}

// ================== QNH Helpers (No NTP) ==================
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
  int i = arr.length()-1; while (i>=0 && (arr[i]==' ' || arr[i]==',')) i--;
  int end=i; while (i>=0 && (isdigit(arr[i]) || arr[i]=='.' || arr[i]==',' || arr[i]=='-')) i--;
  String lastnum = arr.substring(i+1, end+1); lastnum.replace(",", "");
  val = lastnum.toFloat();
  return true;
}
static bool httpGetOpenMeteo(const String& path, String& resp){
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) {
    delay(50); bsecLoopTick(); yield();
  }
  if (WiFi.status() != WL_CONNECTED) { WiFi.mode(WIFI_OFF); return false; }
  WiFiClient client;
  if (!client.connect("api.open-meteo.com", 80)) {
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
    return false;
  }
  client.print(String("GET ") + path + " HTTP/1.0\r\nHost: api.open-meteo.com\r\nConnection: close\r\n\r\n");
  resp = ""; resp.reserve(768);
  unsigned long t1 = millis();
  while ((client.connected() || client.available()) && millis() - t1 < QNH_FETCH_TIMEOUT_MS) {
    while (client.available()) { resp += (char)client.read(); }
    delay(10); bsecLoopTick(); yield();
  }
  client.stop();
  WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
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

// ================== Serial ==================
void handleSerialInput(){
  // Tidak ada perintah khusus saat ini (brightness dihapus).
  // Biarkan stub untuk masa depan (mis. debug).
  while (Serial.available()){ char c=Serial.read(); (void)c; }
}

// ================== Setup/Loop ==================
void setup() {
  EEPROM.begin(EEPROM_SIZE);
  loadPersistent();

  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
  Wire.setClock(100000);
  delay(80);

  initOLED();
  displayInitTwoLines();
  delay(150);

  if (!initBSEC()){
    currentAppMode = MODE_BME_ERROR;
    displayCenteredStatus("BSEC/BME Fail!","Error Mode");
    // jadwalkan retry
    sensorRetryBackoffMs = 1000;
    nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
  }
  Wire.setClock(400000);

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
  bootMs = millis();

  if (currentAppMode == MODE_OFFLINE) readBME680SensorData();

  Serial.begin(115200); // optional: untuk log dasar
  Serial.println(F("Boot OK"));
}

void loop() {
  bsecLoopTick();

  if (currentAppMode == MODE_OFFLINE) {
    if (thermal == THERM_HOT_HOLD) {
      if (millis() >= nextSafetyProcessMs) {
        if (gTemp <= HOT_EXIT_C) { exitHotHold(); }
        if (thermal == THERM_HOT_HOLD) displayOverheat();
        nextSafetyProcessMs = millis() + SAFETY_PERIOD_MS;
      }
    } else {
      if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
        readBME680SensorData();
        lastSensorReadMillis = millis();
      }
      updateOLEDDisplayContent();
    }
  } else {
    handleSensorAutoRetry();
  }

  handleSerialInput();
  yield();
}
