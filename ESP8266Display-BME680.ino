#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SSD1306Wire.h>

#define OLED_ADDRESS 0x3C
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define BME_ADDRESS 0x76

const unsigned long SENSOR_READ_INTERVAL_MS = 30000;
const unsigned long OLED_DATA_SCREEN_1_DURATION_MS = 7000;
const unsigned long OLED_DATA_SCREEN_2_DURATION_MS = 3000;
const unsigned long OLED_OFF_DURATION_MS = SENSOR_READ_INTERVAL_MS - (OLED_DATA_SCREEN_1_DURATION_MS + OLED_DATA_SCREEN_2_DURATION_MS);

const unsigned long BME680_RELIABLE_READ_RETRIES = 5;
const unsigned long BME680_RELIABLE_READ_RETRY_DELAY_MS = 50;
const unsigned long BME680_INIT_RETRY_INTERVAL_MS = 3000;
const int BME680_MAX_INIT_ATTEMPTS = 50;

SSD1306Wire display(OLED_ADDRESS, I2C_SDA_GPIO, I2C_SCL_GPIO);
Adafruit_BME680 bme(&Wire);

char oledBuffer[64]; 

float gTemp = 0.0;
float gHum = 0.0;
float gPress = 0.0;
float gAlt = 0.0;
float gGasResistance = 0.0;
const float seaLevelPressure_hPa_current = 1013.25;

enum AppMode { MODE_OFFLINE, MODE_BME_ERROR };
AppMode currentAppMode = MODE_OFFLINE;

enum OledDisplayState { OLED_STATE_ERROR_SCREEN, OLED_STATE_DATA_SCREEN_1, OLED_STATE_DATA_SCREEN_2, OLED_STATE_OFF };
OledDisplayState currentOledScreenState = OLED_STATE_DATA_SCREEN_1;

unsigned long lastSensorReadMillis = 0;
unsigned long oledScreenStateChangeMillis = 0;

void initOLED();
void initBME680();
void readBME680SensorData();
bool readBME680SensorReliably(float &temp, float &hum, float &press_Pa, float &alt, float &gas);
void updateOLEDDisplayContent();
void displayCenteredStatus(const String& line1, const String& line2);
void displaySensorDataScreen1();
void displaySensorDataScreen2();
String getGasStatus(float gasResistance);

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA_GPIO, I2C_SCL_GPIO);
  Wire.setClock(400000);

  initOLED();
  displayCenteredStatus("BME680 Monitor", "Initializing...");
  delay(1000);

  initBME680();
  if (!bme.begin(BME_ADDRESS)) {
    Wire.setClock(100000);
    initBME680();
  }

  lastSensorReadMillis = millis();
  oledScreenStateChangeMillis = millis();
  readBME680SensorData(); 
}

void loop() {
  if (currentAppMode == MODE_OFFLINE) {
    if (millis() - lastSensorReadMillis >= SENSOR_READ_INTERVAL_MS) {
      readBME680SensorData();
      lastSensorReadMillis = millis();
    }
  } else if (currentAppMode == MODE_BME_ERROR) {
    yield();
  }

  updateOLEDDisplayContent();
  yield();
}

void initOLED() {
  if (!display.init()) {
    while (1) { yield(); }
  } else {
    display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_16);
  }
}

void initBME680() {
  int bmeInitAttemptsCount = 0;
  while (!bme.begin(BME_ADDRESS)) {
    bmeInitAttemptsCount++;
    displayCenteredStatus("BME680 Init Fail!", "Retrying " + String(bmeInitAttemptsCount));
    delay(BME680_INIT_RETRY_INTERVAL_MS);
    if (bmeInitAttemptsCount >= BME680_MAX_INIT_ATTEMPTS) {
      currentAppMode = MODE_BME_ERROR;
      displayCenteredStatus("BME680 Fail!", "Error Mode");
      return;
    }
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);
}

void readBME680SensorData() {
  float temperature, humidity, pressure_Pa, altitude, gas_resistance;
  if (!readBME680SensorReliably(temperature, humidity, pressure_Pa, altitude, gas_resistance)) {
    currentAppMode = MODE_BME_ERROR;
    displayCenteredStatus("Sensor Read Fail!", "Check Conn.!");
    return;
  }
  gTemp = temperature;
  gHum = humidity;
  gPress = pressure_Pa / 100.0F;
  gAlt = altitude;
  gGasResistance = gas_resistance;
  oledScreenStateChangeMillis = millis();
  currentOledScreenState = OLED_STATE_DATA_SCREEN_1; 
  display.displayOn(); 
}

bool readBME680SensorReliably(float &temp, float &hum, float &press_Pa, float &alt, float &gas) {
  for (int i = 0; i < BME680_RELIABLE_READ_RETRIES; i++) {
    if (bme.performReading()) {
      temp = bme.temperature;
      hum = bme.humidity;
      press_Pa = bme.pressure;
      alt = bme.readAltitude(seaLevelPressure_hPa_current);
      gas = bme.gas_resistance / 1000.0;
      if (!isnan(temp) && !isnan(hum) && !isnan(press_Pa) && !isnan(alt) && !isnan(gas) &&
          temp >= -40.0 && temp <= 85.0 &&
          hum >= 0.0 && hum <= 100.0 &&
          press_Pa >= 30000 && press_Pa <= 110000 &&
          gas >= 0.0) {
        return true;
      }
    }
    delay(BME680_RELIABLE_READ_RETRY_DELAY_MS);
  }
  return false;
}

void updateOLEDDisplayContent() {
  if (currentAppMode == MODE_BME_ERROR) {
    return;
  }

  unsigned long currentMillis = millis();
  unsigned long timeSinceLastStateChange = currentMillis - oledScreenStateChangeMillis;

  switch (currentOledScreenState) {
    case OLED_STATE_DATA_SCREEN_1:
      if (timeSinceLastStateChange >= OLED_DATA_SCREEN_1_DURATION_MS) {
        currentOledScreenState = OLED_STATE_DATA_SCREEN_2;
        oledScreenStateChangeMillis = currentMillis;
      }
      break;
    case OLED_STATE_DATA_SCREEN_2:
      if (timeSinceLastStateChange >= OLED_DATA_SCREEN_2_DURATION_MS) {
        currentOledScreenState = OLED_STATE_OFF;
        oledScreenStateChangeMillis = currentMillis;
      }
      break;
    case OLED_STATE_OFF:
      if (timeSinceLastStateChange >= OLED_OFF_DURATION_MS) {
      }
      break;
    case OLED_STATE_ERROR_SCREEN:
      break;
  }

  display.clear();
  if (currentOledScreenState == OLED_STATE_ERROR_SCREEN) {
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 15, "Sensor Error");
    display.drawString(64, 40, "Check Conn.!");
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_1) {
    displaySensorDataScreen1();
  } else if (currentOledScreenState == OLED_STATE_DATA_SCREEN_2) {
    displaySensorDataScreen2();
  } else if (currentOledScreenState == OLED_STATE_OFF) {
    display.displayOff();
  }
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
  snprintf(oledBuffer, sizeof(oledBuffer), "Gas: %.2f kOhm", gGasResistance);
  display.drawString(0, 0, oledBuffer);
  snprintf(oledBuffer, sizeof(oledBuffer), "Status: %s", getGasStatus(gGasResistance).c_str());
  display.drawString(0, 16, oledBuffer);
}

String getGasStatus(float gasResistance) {
  if (gasResistance < 60.0) return "Very Good";
  else if (gasResistance <= 120.0) return "Good";
  else return "Bad";
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