#include "storage.h"
#include "../globals.h"

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
      seaLevelPressure_hPa_current < 870.0f ||
      seaLevelPressure_hPa_current > 1100.0f)
  {
    seaLevelPressure_hPa_current = 1012.50f;
    EEPROM.put(SEA_LEVEL_PRESSURE_ADDR, seaLevelPressure_hPa_current);
    EEPROM.commit();
  }

  EEPROM.get(GAS_BASELINE_ADDR, gasBaseline_kOhm);
  uint8_t rdy = 0;
  EEPROM.get(GAS_BASELINE_READY_ADDR, rdy);
  gasBaselineReady = (rdy != 0);
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
