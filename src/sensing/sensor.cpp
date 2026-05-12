#include "sensor.h"
#include "../globals.h"
#include "../utils.h"
#include "storage.h"
#include "../display/oled.h"

bool initBSEC()
{
  Wire.beginTransmission(BME_ADDRESS);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(10);

  iaqSensor.begin(BME_ADDRESS, Wire);
  if (iaqSensor.bsecStatus < BSEC_OK || iaqSensor.bme68xStatus != BME68X_OK)
  {
    return false;
  }

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

  loadBsecState();
  lastBsecSaveMs = millis();
  gIAQaccPrev = iaqSensor.iaqAccuracy;
  gIAQaccDisp = gIAQaccPrev;

  bsecActive = true;
  bsecHasData = false;
  return true;
}

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

void setQNH(float qnh)
{
  if (qnh >= 870.0f && qnh <= 1100.0f)
  {
    seaLevelPressure_hPa_current = qnh;
    saveSeaLevelPressure(qnh);
    resetAltitudeFiltering();
  }
}

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

void bsecLoopTick()
{
  if (!bsecActive)
    return;
  if (!iaqSensor.run())
    return;

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

  float med = (altCnt >= 3) ? med3(altRaw3[0], altRaw3[1], altRaw3[2]) : alt_raw;
  if (fabsf(alt_raw - med) > ALT_OUTLIER_M)
  {
    alt_raw = med;
  }

  if (isnan(gAltSmooth))
  {
    gAltSmooth = alt_raw;
  }
  else
  {
    float cand = ALT_EMA_ALPHA * alt_raw + (1.0f - ALT_EMA_ALPHA) * gAltSmooth;
    if (fabsf(cand - gAltSmooth) >= ALT_DEADBAND_M)
    {
      gAltSmooth = cand;
    }
  }
  gAlt = gAltSmooth;

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
  iaqMeanEWMA += IAQ_VAR_ALPHA * d;
  iaqVarEWMA = (1.0f - IAQ_VAR_ALPHA) * (iaqVarEWMA + IAQ_VAR_ALPHA * d * d);
  float vol = sqrtf(fmaxf(iaqVarEWMA, 0.0f));

  float aVar;
  if (vol <= 2.0f)
    aVar = 0.10f;
  else if (vol >= 25.0f)
    aVar = 0.45f;
  else
    aVar = 0.10f + (vol - 2.0f) * (0.35f / (25.0f - 2.0f));

  if (gIAQacc >= 3)
    aVar *= 0.6f;
  if (gIAQacc <= 1)
    aVar = fmaxf(aVar, 0.28f);
  aVar = fminf(fmaxf(aVar, 0.08f), 0.45f);

  if (isnan(gIAQstaticDisp))
  {
    gIAQstaticDisp = gIAQstatic;
  }
  else
  {
    gIAQstaticDisp = aVar * gIAQstatic + (1.0f - aVar) * gIAQstaticDisp;
  }

  // Periodic BSEC state save
  if (millis() - lastBsecSaveMs >= BSEC_SAVE_INTERVAL_MS && gIAQacc >= 3)
  {
    saveBsecState();
    lastBsecSaveMs = millis();
  }

  if (gIAQacc > gIAQaccPrev && gIAQacc >= 2)
  {
    if (millis() - lastBsecSaveMs >= BSEC_MIN_SAVE_GAP_MS)
    {
      saveBsecState();
      lastBsecSaveMs = millis();
    }
  }

  gIAQaccDisp = (gIAQacc >= 3) ? 3 : (gIAQacc == 2 && gIAQaccDisp == 3 ? 2 : gIAQacc);
  gIAQaccPrev = gIAQacc;

  // Transport detection
  if (++portDecim >= PORT_DECIM_N)
  {
    portDecim = 0;
    portBufP[portW] = gPress;
    portBufH[portW] = gHum;
    portBufI[portW] = gIAQstatic;
    portW = (portW + 1) % PORT_BUF;
    if (portFill < PORT_BUF)
      portFill++;

    if (portFill >= 2)
    {
      int newest = (portW + PORT_BUF - 1) % PORT_BUF;
      int oldest = (portFill == PORT_BUF) ? portW : 0;
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

void readBME680SensorData()
{
  if (!bsecActive)
    return;

  unsigned long noDataTimeout = (millis() - bootMs < BOOT_GRACE_MS) ? NO_DATA_TIMEOUT_BOOT_MS : NO_DATA_TIMEOUT_RUN_MS;
  if (!bsecHasData || (millis() - lastBsecDataMs > noDataTimeout))
  {
    currentAppMode = MODE_BME_ERROR;
    errorScreenDrawn = false;
    display.clearDisplay();
    display.display();
    sensorRetryBackoffMs = 1000;
    nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
    return;
  }

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

void enterHotHold()
{
  thermal = THERM_HOT_HOLD;
  oledSetContrast(CONTRAST_OVERHEAT);
  displayOverheat();
  nextSafetyProcessMs = millis();
}

void exitHotHold()
{
  thermal = THERM_NORMAL;
  oledSetContrast(CONTRAST_NORMAL);
  lastDrawnState = OLED_STATE_ERROR_SCREEN;
}

void scheduleSensorRetryInitial()
{
  sensorRetryBackoffMs = 1000;
  nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
}

void handleSensorAutoRetry()
{
  if (millis() < nextSensorRetryMillis)
    return;

  if (initBSEC())
  {
    currentAppMode = MODE_OFFLINE;
    errorScreenDrawn = false;
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

  if (sensorRetryBackoffMs < 60000)
    sensorRetryBackoffMs *= 2;
  if (sensorRetryBackoffMs > 60000)
    sensorRetryBackoffMs = 60000;
  nextSensorRetryMillis = millis() + sensorRetryBackoffMs;
}
