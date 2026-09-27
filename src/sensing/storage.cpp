#include "storage.h"
#include "../globals.h"

static void writeWifiFlags();

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

void loadPersistent()
{
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC_VALUE)
  {
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
      seaLevelPressure_hPa_current < QNH_MIN_HPA ||
      seaLevelPressure_hPa_current > QNH_MAX_HPA)
  {
    seaLevelPressure_hPa_current = 1012.50f;
    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.commit();
  }

  EEPROM.get(GAS_BASELINE_ADDR, gasBaseline_kOhm);
  uint8_t rdy = 0;
  EEPROM.get(GAS_BASELINE_READY_ADDR, rdy);
  gasBaselineReady = (rdy != 0);

  loadWifiConfig();
}

static void readEepromStr(int addr, char *dst, size_t cap)
{
  for (size_t i = 0; i < cap; i++)
  {
    EEPROM.get(addr + (int)i, dst[i]);
    if (dst[i] == '\0')
      break;
  }
  dst[cap - 1] = '\0';
}

static int slotSsidAddr(uint8_t slot)
{
  if (slot == 1)
    return WIFI2_SSID_ADDR;
  if (slot == 2)
    return WIFI3_SSID_ADDR;
  return WIFI_SSID_ADDR;
}

static int slotPassAddr(uint8_t slot)
{
  if (slot == 1)
    return WIFI2_PASS_ADDR;
  if (slot == 2)
    return WIFI3_PASS_ADDR;
  return WIFI_PASS_ADDR;
}

void loadWifiConfig()
{
  uint32_t magic = 0;
  EEPROM.get(WIFI_MAGIC_ADDR, magic);

  if (magic != WIFI_MAGIC_VALUE)
  {
    // First run: empty credentials, defaults seeded later by wifiQnhBegin()
    memset(wifiNets, 0, sizeof(wifiNets));
    wifiLat = DEFAULT_LAT;
    wifiLon = DEFAULT_LON;
    qnhAutoEnabled = true;
    qnhSource = QNH_SOURCE_DEFAULT;
    return;
  }

  for (uint8_t i = 0; i < WIFI_MAX_NETS; i++)
  {
    readEepromStr(slotSsidAddr(i), wifiNets[i].ssid, WIFI_SSID_LEN);
    readEepromStr(slotPassAddr(i), wifiNets[i].pass, WIFI_PASS_LEN);
    if (wifiNets[i].ssid[0] == '\0')
      wifiNets[i].pass[0] = '\0'; // orphan password without SSID is unusable
  }
  EEPROM.get(WIFI_LAT_ADDR, wifiLat);
  EEPROM.get(WIFI_LON_ADDR, wifiLon);
  uint8_t flags = 0;
  EEPROM.get(WIFI_FLAGS_ADDR, flags);
  qnhAutoEnabled = (flags & WIFI_FLAG_AUTO) != 0;
  if ((flags & WIFI_FLAG_AUTOSRC) != 0)
    qnhSource = QNH_SOURCE_AUTO;

  if (!(wifiLat >= -90.0f && wifiLat <= 90.0f))
    wifiLat = DEFAULT_LAT;
  if (!(wifiLon >= -180.0f && wifiLon <= 180.0f))
    wifiLon = DEFAULT_LON;
}

void saveWifiConfig()
{
  for (uint8_t i = 0; i < WIFI_MAX_NETS; i++)
  {
    for (size_t j = 0; j < WIFI_SSID_LEN; j++)
      EEPROM.put(slotSsidAddr(i) + (int)j, wifiNets[i].ssid[j]);
    for (size_t j = 0; j < WIFI_PASS_LEN; j++)
      EEPROM.put(slotPassAddr(i) + (int)j, wifiNets[i].pass[j]);
  }
  EEPROM.put(WIFI_LAT_ADDR, wifiLat);
  EEPROM.put(WIFI_LON_ADDR, wifiLon);
  writeWifiFlags();
  EEPROM.put(WIFI_MAGIC_ADDR, (uint32_t)WIFI_MAGIC_VALUE);
  EEPROM.commit();
}

void clearWifiConfig()
{
  memset(wifiNets, 0, sizeof(wifiNets));
  saveWifiConfig();
}

static void writeWifiFlags()
{
  uint8_t flags = 0;
  if (qnhAutoEnabled)
    flags |= WIFI_FLAG_AUTO;
  if (qnhSource == QNH_SOURCE_AUTO)
    flags |= WIFI_FLAG_AUTOSRC;
  EEPROM.put(WIFI_FLAGS_ADDR, flags);
  EEPROM.commit();
}

void saveQnhAuto(bool enabled)
{
  qnhAutoEnabled = enabled;
  writeWifiFlags();
}

void saveQnhSource()
{
  writeWifiFlags();
}

void saveSeaLevelPressure(float p)
{
  EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, p);
  EEPROM.commit();
}

void saveGasBaseline(float b, bool rdy)
{
  EEPROM.put(GAS_BASELINE_ADDR, b);
  EEPROM.put(GAS_BASELINE_READY_ADDR, (uint8_t)rdy);
  EEPROM.commit();
}
