#include "oled.h"
#include "../globals.h"
#include "../utils.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>

void initOLED()
{
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.setTextWrap(false);
  display.display();
  oledSetContrast(CONTRAST_NORMAL);
}

void oledSetContrast(uint8_t v)
{
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(v);
}

static void drawThermometerIcon(int16_t x, int16_t y)
{
  display.fillCircle(x + 6, y + 18, 6, SSD1306_WHITE);
  display.fillRect(x + 5, y, 3, 18, SSD1306_WHITE);
}

static void getOverheatPos(uint8_t idx, int16_t &x, int16_t &y)
{
  static const int8_t pos[5][2] = {
      {10, 10},
      {32, 6},
      {20, 28},
      {48, 18},
      {8, 36}};
  x = pos[idx % 5][0];
  y = pos[idx % 5][1];
}

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

void displayScreen1_TempHumid()
{
  int16_t x1, y1;
  uint16_t w1, h1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setFont(&FreeSans9pt7b);
  display.getTextBounds(F("Temp & Humid"), 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 12);
  display.print(F("Temp & Humid"));
  display.drawFastHLine(0, 15, SCREEN_W, SSD1306_WHITE);

  display.setFont(&FreeSans12pt7b);
  snprintf(oledBuffer, sizeof(oledBuffer), "%.2f C", envData.temperature);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 38);
  display.print(oledBuffer);
  display.drawCircle(((SCREEN_W - w1) / 2) + w1 - 15, 25, 2, SSD1306_WHITE);

  snprintf(oledBuffer, sizeof(oledBuffer), "%.2f %%", envData.humidity);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 62);
  display.print(oledBuffer);

  display.display();
}

void displayScreen2_PressureAlt()
{
  int16_t x1, y1;
  uint16_t w1, h1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setFont(&FreeSans9pt7b);
  display.getTextBounds(F("Press & Altitude"), 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 12);
  display.print(F("Press & Altitude"));
  display.drawFastHLine(0, 15, SCREEN_W, SSD1306_WHITE);

  display.setFont(&FreeSans12pt7b);
  snprintf(oledBuffer, sizeof(oledBuffer), "%.2f hPa", envData.pressure);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 38);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "%d mdpl", (int)lroundf(envData.altitude));
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 60);
  display.print(oledBuffer);

  display.display();
}

void displayScreen3_GasIAQ()
{
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setFont(&FreeSans9pt7b);

  snprintf(oledBuffer, sizeof(oledBuffer), "G: %.1f kOhm", envData.gasResistanceEMA);
  display.setCursor(0, 14);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "IAQ: %.1f", envData.iaqStaticDisp);
  display.setCursor(0, 30);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "Acc: %u", envData.iaqAccuracyDisp);
  display.setCursor(0, 46);
  display.print(oledBuffer);

  snprintf(oledBuffer, sizeof(oledBuffer), "AQS: %s", getIaqCategory(envData.iaqStaticDisp));
  display.setCursor(0, 60);
  display.print(oledBuffer);

  display.display();
}

void displayScreen4_Uptime()
{
  int16_t x1, y1;
  uint16_t w1, h1;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setFont(&FreeSans9pt7b);
  display.getTextBounds(F("Uptime"), 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 12);
  display.print(F("Uptime"));
  display.drawFastHLine(0, 15, SCREEN_W, SSD1306_WHITE);

  unsigned long nowMs = millis();
  unsigned long elapsed = (nowMs >= bootMs) ? (nowMs - bootMs) : nowMs;
  unsigned long totalSec = elapsed / 1000;
  unsigned long h = totalSec / 3600;
  unsigned long m = (totalSec % 3600) / 60;
  unsigned long s = totalSec % 60;

  display.setFont(&FreeSans9pt7b);
  snprintf(oledBuffer, sizeof(oledBuffer), "%02lu:%02lu:%02lu", h, m, s);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 34);
  display.print(oledBuffer);

  // NOTE: default font draws DOWNWARD from cursor (8px tall).
  // Line 1 (y=42, rows 42..49): live QNH value.
  // Line 2 (y=54, rows 54..61): sync age / state.
  display.setFont();
  snprintf(oledBuffer, sizeof(oledBuffer), "QNH %.1f", seaLevelPressure_hPa_current);
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 42);
  display.print(oledBuffer);

  if (wifiSsid[0] != '\0' && qnhSource != QNH_SOURCE_MANUAL)
  {
    if (lastQnhSyncMs)
    {
      unsigned long ageMin = (nowMs - lastQnhSyncMs) / 60000UL;
      if (ageMin < 1)
        snprintf(oledBuffer, sizeof(oledBuffer), "SYNC NOW");
      else if (ageMin < 60)
        snprintf(oledBuffer, sizeof(oledBuffer), "SYNC %lum AGO", ageMin);
      else
        snprintf(oledBuffer, sizeof(oledBuffer), "SYNC %luh AGO", (ageMin + 30) / 60);
    }
    else if (wifiFailCount > 0)
    {
      snprintf(oledBuffer, sizeof(oledBuffer), "SYNC FAIL");
    }
    else
    {
      snprintf(oledBuffer, sizeof(oledBuffer), "WIFI WAIT");
    }
  }
  else if (qnhSource == QNH_SOURCE_MANUAL)
  {
    snprintf(oledBuffer, sizeof(oledBuffer), "MANUAL");
  }
  else
  {
    snprintf(oledBuffer, sizeof(oledBuffer), "QNH DEF");
  }
  display.getTextBounds(oledBuffer, 0, 0, &x1, &y1, &w1, &h1);
  display.setCursor((SCREEN_W - w1) / 2, 54);
  display.print(oledBuffer);

  display.display();
}

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

static bool shouldRedrawScreen1()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  if (isnan(prev_T) || isnan(prev_H))
    return true;
  return hasChanged(envData.temperature, prev_T, TH_T) || hasChanged(envData.humidity, prev_H, TH_H);
}

static bool shouldRedrawScreen2()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  if (isnan(prev_P) || isnan(prev_Alt))
    return true;
  return hasChanged(envData.pressure, prev_P, TH_P) || hasChanged(envData.altitude, prev_Alt, TH_ALT);
}

static bool shouldRedrawScreen3()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  if (isnan(prev_G) || isnan(prev_IAQ))
    return true;
  return hasChanged(envData.gasResistanceEMA, prev_G, TH_G) ||
         hasChanged(envData.iaqStaticDisp, prev_IAQ, TH_IAQ) ||
         envData.iaqAccuracyDisp != prev_Acc ||
         prev_AQS != getIaqCategory(envData.iaqStaticDisp);
}

static QnhSource prevQnhSrc = QNH_SOURCE_DEFAULT;
static unsigned long prevSyncMin = 0;
static uint8_t prevFail = 0;
static float prevQnhVal = NAN;

static unsigned long qnhSyncAgeMin()
{
  return lastQnhSyncMs ? (millis() - lastQnhSyncMs) / 60000UL : 0;
}

static bool shouldRedrawScreen4()
{
  if (currentOledScreenState != lastDrawnState)
    return true;
  unsigned long nowSec = millis() / 1000;
  return nowSec != prevUptimeSec ||
         qnhSource != prevQnhSrc ||
         qnhSyncAgeMin() != prevSyncMin ||
         wifiFailCount != prevFail ||
         seaLevelPressure_hPa_current != prevQnhVal;
}

static void stampScreen1()
{
  prev_T = envData.temperature;
  prev_H = envData.humidity;
  lastDrawnState = OLED_STATE_DATA_SCREEN_1;
}

static void stampScreen2()
{
  prev_P = envData.pressure;
  prev_Alt = envData.altitude;
  lastDrawnState = OLED_STATE_DATA_SCREEN_2;
}

static void stampScreen3()
{
  prev_G = envData.gasResistanceEMA;
  prev_IAQ = envData.iaqStaticDisp;
  prev_Acc = envData.iaqAccuracyDisp;
  prev_AQS = getIaqCategory(envData.iaqStaticDisp);
  lastDrawnState = OLED_STATE_DATA_SCREEN_3;
}

static void stampScreen4()
{
  prevUptimeSec = millis() / 1000;
  prevQnhSrc = qnhSource;
  prevSyncMin = qnhSyncAgeMin();
  prevFail = wifiFailCount;
  prevQnhVal = seaLevelPressure_hPa_current;
  lastDrawnState = OLED_STATE_DATA_SCREEN_4;
}

void updateOLEDDisplayContent()
{
  if (currentAppMode == MODE_BME_ERROR)
  {
    if (!errorScreenDrawn)
    {
      displayErrorScreen();
      errorScreenDrawn = true;
    }
    return;
  }

  if (thermal == THERM_HOT_HOLD)
  {
    if (millis() >= nextSafetyProcessMs)
    {
      displayOverheat();
      nextSafetyProcessMs = millis() + SAFETY_PERIOD_MS;
    }
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - oledScreenStateChangeMillis;

  switch (currentOledScreenState)
  {
  case OLED_STATE_DATA_SCREEN_1:
    if (elapsed >= OLED_DATA_SCREEN_1_DURATION)
    {
      currentOledScreenState = OLED_STATE_DATA_SCREEN_2;
      oledScreenStateChangeMillis = now;
      lastDrawnState = OLED_STATE_ERROR_SCREEN;
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
    currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
    oledScreenStateChangeMillis = now;
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
    break;
  }
}
