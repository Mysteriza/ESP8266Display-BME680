#include "serial_cmd.h"
#include "../globals.h"
#include "../sensing/sensor.h"
#include "../sensing/storage.h"
#include "wifi_qnh.h"

static void printWifiStatus()
{
  const char *src = (qnhSource == QNH_SOURCE_AUTO) ? "AUTO" : (qnhSource == QNH_SOURCE_MANUAL) ? "MAN" : "DEF";
  Serial.printf("WIFI ssid=%s auto=%c src=%s ",
                wifiSsid[0] ? wifiSsid : "-",
                qnhAutoEnabled ? 'Y' : 'N', src);
  if (lastQnhSyncMs)
    Serial.printf("sync=%lum ago ", (millis() - lastQnhSyncMs) / 60000UL);
  else
    Serial.print(F("sync=never "));
  Serial.printf("rc=%d st=%d fail=%u lat=%.4f lon=%.4f\r\n",
                wifiLastHttpCode, wifiLastStatus, wifiFailCount, wifiLat, wifiLon);
}

void handleSerialInput()
{
  static char line[128];
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
          Serial.printf("OK QNH=%.2f hPa (MAN, auto off)\r\n", seaLevelPressure_hPa_current);
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
          Serial.printf("OK QNH=%.2f hPa (MAN, auto off)\r\n", seaLevelPressure_hPa_current);
        }
        else if (strcasecmp(line, "STATUS") == 0)
        {
          Serial.printf("Mode:%d Therm:%d BSEC:%s IAQ:%.1f(%u)\r\n",
                        currentAppMode, thermal, bsecActive ? "Y" : "N",
                        envData.iaqStaticDisp, envData.iaqAccuracyDisp);
        }
        else if (strncmp(line, "WIFI_SSID=", 10) == 0)
        {
          size_t n = strlen(line + 10);
          if (n >= 1 && n < WIFI_SSID_LEN)
          {
            strncpy(wifiSsid, line + 10, WIFI_SSID_LEN - 1);
            wifiSsid[WIFI_SSID_LEN - 1] = '\0';
            saveWifiConfig();
            Serial.printf("OK WIFI_SSID=%s\r\n", wifiSsid);
          }
          else
          {
            Serial.println(F("ERR SSID 1..32 chars"));
          }
        }
        else if (strncmp(line, "WIFI_PASS=", 10) == 0)
        {
          size_t n = strlen(line + 10);
          if (n < WIFI_PASS_LEN)
          {
            strncpy(wifiPass, line + 10, WIFI_PASS_LEN - 1);
            wifiPass[WIFI_PASS_LEN - 1] = '\0';
            saveWifiConfig();
            Serial.println(F("OK WIFI saved"));
          }
          else
          {
            Serial.println(F("ERR PASS max 64 chars"));
          }
        }
        else if (strcasecmp(line, "WIFI?") == 0)
        {
          printWifiStatus();
        }
        else if (strcasecmp(line, "WIFICLEAR") == 0)
        {
          clearWifiConfig();
          Serial.println(F("OK WIFI cleared"));
        }
        else if (strncmp(line, "LAT=", 4) == 0)
        {
          float v = atof(line + 4);
          if (v >= -90.0f && v <= 90.0f)
          {
            wifiLat = v;
            saveWifiConfig();
            Serial.printf("OK LAT=%.6f\r\n", wifiLat);
          }
          else
          {
            Serial.println(F("ERR LAT -90..90"));
          }
        }
        else if (strncmp(line, "LON=", 4) == 0)
        {
          float v = atof(line + 4);
          if (v >= -180.0f && v <= 180.0f)
          {
            wifiLon = v;
            saveWifiConfig();
            Serial.printf("OK LON=%.6f\r\n", wifiLon);
          }
          else
          {
            Serial.println(F("ERR LON -180..180"));
          }
        }
        else if (strncmp(line, "QNHMODE=", 8) == 0)
        {
          if (strcasecmp(line + 8, "AUTO") == 0)
          {
            saveQnhAuto(true);
            Serial.println(F("OK QNHMODE=AUTO"));
          }
          else if (strcasecmp(line + 8, "MAN") == 0)
          {
            saveQnhAuto(false);
            Serial.println(F("OK QNHMODE=MAN"));
          }
          else
          {
            Serial.println(F("ERR use QNHMODE=AUTO|MAN"));
          }
        }
        else if (strcasecmp(line, "SYNCNOW") == 0)
        {
          if (wifiQnhHasCreds())
          {
            wifiQnhForceSync();
            Serial.println(F("OK sync scheduled"));
          }
          else
          {
            Serial.println(F("ERR no wifi creds"));
          }
        }
        else if (strcasecmp(line, "HELP") == 0)
        {
          Serial.println(F("CMD: QNH=<hPa>|QNH?|ALT?|PRESS?|ALTREF=<m>|STATUS|WIFI_SSID=<s>|WIFI_PASS=<p>|WIFI?|WIFICLEAR|LAT=<f>|LON=<f>|QNHMODE=AUTO|MAN|SYNCNOW|HELP"));
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
