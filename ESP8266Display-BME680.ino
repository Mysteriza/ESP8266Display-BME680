// ============================================================================
// ESP8266 BME680 Environmental Monitor
// Modular firmware: config | types | globals | utils | oled | sensor | storage | serial_cmd | wifi_qnh
// ============================================================================

#include <Wire.h>

#include "src/config.h"
#include "src/types.h"
#include "src/globals.h"
#include "src/utils.h"
#include "src/display/oled.h"
#include "src/sensing/storage.h"
#include "src/sensing/sensor.h"
#include "src/communication/serial_cmd.h"
#include "src/communication/wifi_qnh.h"

// ============================================================================
// SETUP
// ============================================================================

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
    errorScreenDrawn = false;
    sensorRetryBackoffMs = 1000;
    nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
    Serial.println(F("BSEC: init failed, will retry"));
  }
  else
  {
    Serial.println(F("BSEC: ready"));
  }

  Wire.setClock(I2C_CLOCK_FAST);
  wifiQnhBegin();
  lastSensorReadMillis = millis();
  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1;
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.display();
  lastDrawnState = OLED_STATE_ERROR_SCREEN;

  Serial.println(F("Type HELP for commands"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  bsecLoopTick();

  if (currentAppMode == MODE_OFFLINE)
  {
    if (thermal == THERM_HOT_HOLD)
    {
      if (millis() >= nextSafetyProcessMs)
      {
        if (envData.temperature <= HOT_EXIT_C)
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
      if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS)
      {
        readBME680SensorData();
        lastSensorReadMillis = millis();
      }
      updateOLEDDisplayContent();
    }
  }
  else if (currentAppMode == MODE_BME_ERROR)
  {
    handleSensorAutoRetry();
  }

  handleSerialInput();
  wifiQnhTick();
  yield();
}
