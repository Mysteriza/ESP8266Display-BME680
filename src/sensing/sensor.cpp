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
  envData.iaqAccuracyPrev = iaqSensor.iaqAccuracy;
  envData.iaqAccuracyDisp = envData.iaqAccuracyPrev;

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

bool setQNH(float qnh)
{
  if (!(qnh >= QNH_MIN_HPA && qnh <= QNH_MAX_HPA))
    return false;
  seaLevelPressure_hPa_current = qnh;
  saveSeaLevelPressure(qnh);
  // Manual calibration wins: freeze auto-sync so field work is never
  // silently overridden by the next hourly fetch. Re-enable via QNHMODE=AUTO.
  qnhSource = QNH_SOURCE_MANUAL;
  saveQnhAuto(false);
  resetAltitudeFiltering();
  return true;
}

bool applyAutoQnh(float qnh)
{
  if (!qnhAutoEnabled)
    return false;
  if (!(qnh >= QNH_MIN_HPA && qnh <= QNH_MAX_HPA))
    return false;
  // Deadband: ignore API jitter below ~3m to keep altitude stable
  if (fabsf(qnh - seaLevelPressure_hPa_current) < QNH_AUTO_DEADBAND_HPA)
    return false;
  seaLevelPressure_hPa_current = qnh;
  saveSeaLevelPressure(qnh);
  qnhSource = QNH_SOURCE_AUTO;
  saveQnhSource();
  resetAltitudeFiltering();
  return true;
}

void calQNHFromAltRef(float href_m)
{
  if (!(href_m > -1000.0f && href_m < 10000.0f))
    return;
  float q = qnhFromRef(envData.pressure, href_m);
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

  envData.iaq = iaqSensor.iaq;
  envData.iaqStatic = iaqSensor.staticIaq;
  envData.iaqAccuracy = iaqSensor.iaqAccuracy;
  envData.temperature = iaqSensor.temperature;
  envData.humidity = iaqSensor.humidity;
  envData.pressure = iaqSensor.pressure / 100.0f;
  envData.gasResistance = iaqSensor.gasResistance / 1000.0f;

  // Altitude calculation with median + EMA filtering
  float alt_raw = temperatureCompensatedAltitude(envData.pressure, seaLevelPressure_hPa_current, envData.temperature);
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
  envData.altitude = gAltSmooth;

  if (!bsecHasData)
    lastDrawnState = OLED_STATE_ERROR_SCREEN;
  bsecHasData = true;
  lastBsecDataMs = millis();

  // Gas resistance EMA filtering
  if (isnan(envData.gasResistanceEMA))
  {
    envData.gasResistanceEMA = envData.gasResistance;
  }
  else
  {
    envData.gasResistanceEMA = GAS_EMA_ALPHA * envData.gasResistance + (1.0f - GAS_EMA_ALPHA) * envData.gasResistanceEMA;
  }

  // IAQ variance-adaptive smoothing
  if (isnan(iaqMeanEWMA))
  {
    iaqMeanEWMA = envData.iaqStatic;
    iaqVarEWMA = 0.0f;
  }
  float d = envData.iaqStatic - iaqMeanEWMA;
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

  if (envData.iaqAccuracy >= 3)
    aVar *= 0.6f;
  if (envData.iaqAccuracy <= 1)
    aVar = fmaxf(aVar, 0.28f);
  aVar = fminf(fmaxf(aVar, 0.08f), 0.45f);

  if (isnan(envData.iaqStaticDisp))
  {
    envData.iaqStaticDisp = envData.iaqStatic;
  }
  else
  {
    envData.iaqStaticDisp = aVar * envData.iaqStatic + (1.0f - aVar) * envData.iaqStaticDisp;
  }

  // Periodic BSEC state save
  if (millis() - lastBsecSaveMs >= BSEC_SAVE_INTERVAL_MS && envData.iaqAccuracy >= 3)
  {
    saveBsecState();
    lastBsecSaveMs = millis();
  }

  if (envData.iaqAccuracy > envData.iaqAccuracyPrev && envData.iaqAccuracy >= 2)
  {
    if (millis() - lastBsecSaveMs >= BSEC_MIN_SAVE_GAP_MS)
    {
      saveBsecState();
      lastBsecSaveMs = millis();
    }
  }

  envData.iaqAccuracyDisp = (envData.iaqAccuracy >= 3) ? 3 : (envData.iaqAccuracy == 2 && envData.iaqAccuracyDisp == 3 ? 2 : envData.iaqAccuracy);
  envData.iaqAccuracyPrev = envData.iaqAccuracy;

  // Transport detection
  if (++portDecim >= PORT_DECIM_N)
  {
    portDecim = 0;
    portBufP[portW] = envData.pressure;
    portBufH[portW] = envData.humidity;
    portBufI[portW] = envData.iaqStatic;
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
      gasBaseline_kOhm = envData.gasResistanceEMA;
    }
    float dv = envData.gasResistanceEMA - gasBaseline_kOhm;
    gasBaseline_kOhm += (dv > 0 ? BASELINE_ALPHA_UP : BASELINE_ALPHA_DOWN) * dv;
  }

  gasSampleCounter++;
  if (!gasBaselineReady && gasSampleCounter >= GAS_BASELINE_READY_SAMPLES)
  {
    gasBaselineReady = true;
    saveGasBaseline(gasBaseline_kOhm, true);
  }

  if (thermal == THERM_NORMAL && envData.temperature >= HOT_ENTER_C)
  {
    enterHotHold();
    return;
  }
  if (thermal == THERM_HOT_HOLD && envData.temperature <= HOT_EXIT_C)
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
