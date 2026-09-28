#include "serial_cmd.h"
#include "../globals.h"
#include "../sensing/sensor.h"

void handleSerialInput()
{
  static char line[32];
  static uint8_t pos = 0;

  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (pos > 0)
      {
        line[pos] = '\0';

        if (strncmp(line, "QNH=", 4) == 0)
        {
          float qnh = atof(line + 4);
          setQNH(qnh);
          Serial.printf("OK QNH=%.2f hPa\r\n", seaLevelPressure_hPa_current);
        }
        else if (strcasecmp(line, "QNH?") == 0)
        {
          Serial.printf("QNH=%.2f hPa\r\n", seaLevelPressure_hPa_current);
        }
        else if (strcasecmp(line, "ALT?") == 0)
        {
          Serial.printf("ALT=%.2f m\r\n", envData.altitude);
        }
        else if (strcasecmp(line, "PRESS?") == 0)
        {
          Serial.printf("P=%.2f hPa\r\n", envData.pressure);
        }
        else if (strncmp(line, "ALTREF=", 7) == 0)
        {
          float ref = atof(line + 7);
          calQNHFromAltRef(ref);
          Serial.printf("OK QNH=%.2f hPa\r\n", seaLevelPressure_hPa_current);
        }
        else if (strcasecmp(line, "STATUS") == 0)
        {
          Serial.printf("Mode:%d Therm:%d BSEC:%s IAQ:%.1f(%u)\r\n",
                        currentAppMode, thermal, bsecActive ? "Y" : "N",
                        envData.iaqStaticDisp, envData.iaqAccuracyDisp);
        }
        else if (strcasecmp(line, "HELP") == 0)
        {
          Serial.println(F("CMD: QNH=<hPa>|QNH?|ALT?|PRESS?|ALTREF=<m>|STATUS|HELP"));
        }

        pos = 0;
      }
    }
    else if (pos < sizeof(line) - 1)
    {
      line[pos++] = c;
    }
  }
}
