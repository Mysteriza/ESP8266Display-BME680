#include "wifi_qnh.h"
#include "../globals.h"
#include "../sensing/storage.h"
#include "../sensing/sensor.h"
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#ifndef WIFI_DEFAULT_SSID
#define WIFI_DEFAULT_SSID ""
#endif
#ifndef WIFI_DEFAULT_PASS
#define WIFI_DEFAULT_PASS ""
#endif
#ifndef WIFI_DEFAULT_SSID2
#define WIFI_DEFAULT_SSID2 ""
#endif
#ifndef WIFI_DEFAULT_PASS2
#define WIFI_DEFAULT_PASS2 ""
#endif

namespace
{
enum WqState : uint8_t
{
  WQ_IDLE,
  WQ_CONNECTING
};

WqState wqState = WQ_IDLE;
unsigned long wqNextSyncMs = 0;
unsigned long wqConnectStartMs = 0;
uint8_t wqAttempt = 0;
uint8_t wqCand[WIFI_MAX_NETS] = {0};
uint8_t wqCandCount = 0;
uint8_t wqCandPos = 0;

void radioWake()
{
  WiFi.forceSleepWake();
  delay(1);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(WIFI_HOSTNAME);
  // Reduced TX power: prevents USB brownout on marginal supplies,
  // saves battery, still plenty for a home AP in range.
  WiFi.setOutputPower(10.0f);
  yield();
}

void radioSleep()
{
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  yield();
}

// Strip HTTP chunk framing in place (some middleboxes/proxies add it
// even to HTTP/1.0 responses). Plain JSON starts with '{' → fast-path no-op.
// All accesses are clamped to [buf, buf+cap) — never reads/writes past it.
static void dechunkInPlace(char *buf, size_t cap)
{
  if (cap < 4)
    return;
  if (buf[0] == '{')
    return; // fast path: plain JSON
  char *end = nullptr;
  (void)strtoul(buf, &end, 16);
  if (end == buf)
    return; // neither JSON nor chunked: leave for caller to reject

  size_t src = 0, dst = 0;
  while (src + 1 < cap && buf[src])
  {
    unsigned long n = strtoul(buf + src, &end, 16);
    if (end == buf + src)
      break;
    size_t off = (size_t)(end - buf);
    if (off + 1 >= cap)
      break;
    src = off + (end[0] == '\r' ? 2 : 1);
    if (n == 0 || src >= cap - 1)
      break;
    size_t avail = (cap - 1) - src;
    size_t take = (size_t)n < avail ? (size_t)n : avail;
    memmove(buf + dst, buf + src, take);
    dst += take;
    src += take;
    if (take < (size_t)n)
      break; // truncated: keep what we have
    if (src + 1 < cap && buf[src] == '\r')
      src += 2;
    else if (src < cap && buf[src] == '\n')
      src += 1;
  }
  buf[dst < cap ? dst : cap - 1] = '\0';
}

// Returns HTTP status on success path, negative on local failure:
// -1 connect failed, -2 response timeout, -3 parse/validation failed.
int fetchPressureMsl(float *out)
{
  char path[128];
  snprintf(path, sizeof(path),
           "/v1/forecast?latitude=%.6f&longitude=%.6f&current=pressure_msl",
           wifiLat, wifiLon);

  WiFiClient client;
  if (!client.connect(WIFI_API_HOST, 80))
  {
    client.stop();
    return -1;
  }

  client.printf("GET %s HTTP/1.0\r\nHost: " WIFI_API_HOST "\r\nConnection: close\r\n\r\n", path);

  const unsigned long deadline = millis() + WIFI_FETCH_TIMEOUT_MS;

  // Capture status line (first line, max 63 chars)
  char statusLine[64] = {0};
  size_t slPos = 0;
  bool gotStatus = false;
  // Sliding 4-char window to detect end of headers
  char tail[4] = {0};

  while ((long)(millis() - deadline) < 0)
  {
    while (client.available())
    {
      char c = (char)client.read();
      if (!gotStatus)
      {
        if (c == '\n')
        {
          gotStatus = true;
        }
        else if (c != '\r' && slPos < sizeof(statusLine) - 1)
        {
          statusLine[slPos++] = c;
        }
      }
      tail[0] = tail[1];
      tail[1] = tail[2];
      tail[2] = tail[3];
      tail[3] = c;
      if (tail[0] == '\r' && tail[1] == '\n' && tail[2] == '\r' && tail[3] == '\n')
        goto HEADERS_DONE;
    }
    // NOTE: no break on disconnect here — on ESP8266, connected() can read
    // false while the final packet is still in flight. The deadline bounds
    // this loop; late bytes are drained by the inner while above.
    yield();
  }
  client.stop();
  return -2;

HEADERS_DONE:
{
  int code = 0;
  if (sscanf(statusLine, "HTTP/%*d.%*d %d", &code) != 1)
  {
    client.stop();
    return -2;
  }
  if (code != 200)
  {
    client.stop();
    return code;
  }

  static char body[WIFI_BODY_MAX];
  size_t bodyPos = 0;
  const unsigned long bodyDeadline = millis() + WIFI_FETCH_TIMEOUT_MS;
  while (bodyPos < sizeof(body) - 1 && (long)(millis() - bodyDeadline) < 0)
  {
    while (client.available() && bodyPos < sizeof(body) - 1)
    {
      body[bodyPos++] = (char)client.read();
    }
    if (!client.connected() && !client.available())
      break;
    yield();
  }
  client.stop();
  body[bodyPos] = '\0';

  dechunkInPlace(body, sizeof(body));

  // NOTE: "pressure_msl" appears twice: once in current_units as the
  // STRING "hPa", once in current as the numeric value. Scan every
  // occurrence and take the first one that parses as a number in range.
  float v = NAN;
  const char *p = body;
  while ((p = strstr(p, "\"pressure_msl\"")) != nullptr)
  {
    const char *c = strchr(p, ':');
    p += 14; // advance past this key occurrence
    if (!c)
      continue;
    char *end = nullptr;
    float t = strtof(c + 1, &end);
    if (end != c + 1 && t >= QNH_MIN_HPA && t <= QNH_MAX_HPA)
    {
      v = t;
      break;
    }
  }
  if (isnan(v))
  {
    Serial.printf("WiFiQNH: bad value body[%u]: %.96s\r\n", (unsigned)bodyPos, body);
    return -3;
  }

  *out = v;
  return 200;
}
} // end fetchPressureMsl
} // namespace

bool wifiQnhHasCreds()
{
  for (uint8_t i = 0; i < WIFI_MAX_NETS; i++)
    if (wifiNets[i].ssid[0] != '\0')
      return true;
  return false;
}

// Scan surroundings and order stored networks by slot priority.
// Falls back to a blind slot-0 attempt (hidden SSID) when nothing matches.
uint8_t buildCandidates()
{
  wqCandCount = 0;
  int n = WiFi.scanNetworks(false, true);
  if (n < 0)
    n = 0;
  for (uint8_t s = 0; s < WIFI_MAX_NETS && wqCandCount < WIFI_MAX_NETS; s++)
  {
    if (wifiNets[s].ssid[0] == '\0')
      continue;
    for (int i = 0; i < n; i++)
    {
      String ap = WiFi.SSID(i); // short-lived, freed each iteration
      if (ap == wifiNets[s].ssid)
      {
        wqCand[wqCandCount++] = s;
        break;
      }
    }
  }
  WiFi.scanDelete();
  if (wqCandCount == 0 && wifiNets[0].ssid[0] != '\0')
    wqCand[wqCandCount++] = 0; // hidden-SSID fallback
  return wqCandCount;
}

void beginCandidate()
{
  uint8_t slot = wqCand[wqCandPos];
  strncpy(wifiLastAp, wifiNets[slot].ssid, WIFI_SSID_LEN - 1);
  wifiLastAp[WIFI_SSID_LEN - 1] = '\0';
  WiFi.begin(wifiNets[slot].ssid, wifiNets[slot].pass);
  wqConnectStartMs = millis();
}

void wifiQnhForceSync()
{
  if (wifiQnhHasCreds())
    wqNextSyncMs = millis();
}

static void seedSlot(uint8_t slot, const char *ssid, const char *pass)
{
  if (wifiNets[slot].ssid[0] == '\0' && ssid[0] != '\0')
  {
    strncpy(wifiNets[slot].ssid, ssid, WIFI_SSID_LEN - 1);
    wifiNets[slot].ssid[WIFI_SSID_LEN - 1] = '\0';
    strncpy(wifiNets[slot].pass, pass, WIFI_PASS_LEN - 1);
    wifiNets[slot].pass[WIFI_PASS_LEN - 1] = '\0';
  }
}

void wifiQnhBegin()
{
  // Seed RAM from compile-time defaults on first boot (EEPROM empty)
  seedSlot(0, WIFI_DEFAULT_SSID, WIFI_DEFAULT_PASS);
  seedSlot(1, WIFI_DEFAULT_SSID2, WIFI_DEFAULT_PASS2);

  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  radioSleep();

  if (wifiQnhHasCreds())
    wqNextSyncMs = millis() + WIFI_FIRST_SYNC_DELAY_MS;

  Serial.printf("WiFiQNH: %s\r\n", wifiQnhHasCreds() ? "periodic sync armed" : "no creds, offline mode");
}

void wifiQnhTick()
{
  if (!wifiQnhHasCreds())
    return;

  unsigned long now = millis();

  if (wqState == WQ_IDLE)
  {
    if ((long)(now - wqNextSyncMs) < 0)
      return;
    radioWake();
    if (buildCandidates() == 0)
    {
      Serial.println(F("WiFiQNH: no known AP in range"));
      if (wifiFailCount < 255)
        wifiFailCount++;
      wifiLastHttpCode = -1;
      wifiLastStatus = (int)WiFi.status();
      radioSleep();
      wqNextSyncMs = millis() + WIFI_RETRY_FAIL_MS;
      return;
    }
    wqAttempt = 1;
    wqCandPos = 0;
    beginCandidate();
    wqState = WQ_CONNECTING;
    return;
  }

  // WQ_CONNECTING
  if (WiFi.status() == WL_CONNECTED)
  {
    float p = NAN;
    int rc = fetchPressureMsl(&p);
    wifiLastHttpCode = rc;
    if (rc == 200)
    {
      lastQnhSyncMs = millis();
      wifiFailCount = 0;
      if (qnhAutoEnabled && qnhSource != QNH_SOURCE_AUTO)
      {
        // Fresh API validation: record AUTO provenance even when the
        // value is held by deadband or manual-hold policy.
        qnhSource = QNH_SOURCE_AUTO;
        saveQnhSource();
      }
      if (applyAutoQnh(p))
        Serial.printf("WiFiQNH: QNH=%.2f hPa (auto)\r\n", p);
      else
        Serial.printf("WiFiQNH: QNH=%.2f hPa (within deadband/held)\r\n", p);
      wqNextSyncMs = millis() + WIFI_SYNC_INTERVAL_MS;
    }
    else
    {
      if (wifiFailCount < 255)
        wifiFailCount++;
      Serial.printf("WiFiQNH: fetch failed rc=%d\r\n", rc);
      wqNextSyncMs = millis() + WIFI_RETRY_FAIL_MS;
    }
    wqAttempt = 0;
    radioSleep();
    wqState = WQ_IDLE;
    return;
  }

  if (now - wqConnectStartMs >= WIFI_CONNECT_TIMEOUT_MS)
  {
    wifiLastStatus = (int)WiFi.status();
    if (wqAttempt < WIFI_MAX_ATTEMPTS)
    {
      // Next candidate (rotates when several known APs are visible,
      // retries the same one when only one is around) — max 3 tries/min.
      wqAttempt++;
      wqCandPos = (wqCandPos + 1) % wqCandCount;
      Serial.printf("WiFiQNH: try %u/%u '%s' st=%d\r\n",
                    wqAttempt, WIFI_MAX_ATTEMPTS,
                    wifiNets[wqCand[wqCandPos]].ssid, wifiLastStatus);
      WiFi.disconnect();
      delay(WIFI_RETRY_GAP_MS);
      beginCandidate();
    }
    else
    {
      if (wifiFailCount < 255)
        wifiFailCount++;
      wifiLastHttpCode = -1;
      Serial.printf("WiFiQNH: no wifi after %u tries (st=%d)\r\n",
                    wqAttempt, wifiLastStatus);
      wqAttempt = 0;
      wqCandCount = 0;
      radioSleep();
      wqState = WQ_IDLE;
      wqNextSyncMs = millis() + WIFI_RETRY_FAIL_MS;
    }
  }
}
