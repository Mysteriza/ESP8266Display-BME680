# High-Precision ESP8266 BME680 Environmental Monitor

A high-precision, portable environmental monitor using an ESP8266, a Bosch BME680 sensor, and an integrated OLED display. This project leverages the official **Bosch BSEC (Bosch Sensortec Environmental Cluster) software library** to provide accurate Indoor Air Quality (IAQ) readings, alongside temperature, humidity, pressure, and altitude.

The device is designed for **continuous operation** with robust error handling, intelligent baseline management, and an **always-on OLED display** that continuously cycles through data screens.

[![Last Updated](https://img.shields.io/github/last-commit/mysteriza/ESP8266Display-BME680?label=Last%20Updated)](https://github.com/mysteriza/ESP8266Display-BME680/commits/main)

<img src="https://github.com/user-attachments/assets/16ce6f7f-89be-46ac-ae11-b34ff67bfa96" alt="ESP8266 Device Front View" width="400">
<img src="https://github.com/user-attachments/assets/261b5c6e-4504-447e-8210-7e87774dcaeb" alt="ESP8266 Device Side View" width="400">
<img src="https://github.com/user-attachments/assets/99518c65-6359-4fd6-8f37-d91e0dde18bd" alt="ESP8266 Device Rear View" width="400">
<img src="https://github.com/user-attachments/assets/62e0e651-10e9-4774-bc52-773508a8fb07" alt="ESP8266 Device Side View" width="400">

---

## Key Features

### Core Sensing & IAQ

- **Bosch BSEC Integration:** Utilizes Bosch's proprietary BSEC algorithm for reliable **Indoor Air Quality (IAQ)** index calculation (0–500 scale).
- **Comprehensive Data:** Measures Temperature, Humidity, Barometric Pressure, Gas Resistance (VOCs), and calculates Altitude.
- **Data Persistence:** Saves the BSEC calibration state to EEPROM every **4 hours**, but only when the IAQ accuracy level is high (`iaqAccuracy==3`). This minimizes flash wear and ensures stable calibration across power cycles.

### Advanced Data Processing

- **Variance-Aware IAQ Smoothing:** Implements an adaptive smoothing algorithm on the Static IAQ value, providing a more stable and human-readable output.
- **Self-Adapting Gas Baseline:** Automatically adapts over time for long-term environmental stability.
- **Transport-Aware Logic:** Freezes baseline calibration when rapid environmental changes are detected (e.g., device movement).
- **Altitude Filtering:** Combines a median filter with an EMA for smooth and reliable altitude readings.

### Display & Power

- **Continuous Display Mode:** OLED display remains active 24/7, cycling through four data screens:
  - **Screen 1 (5s):** Temperature & Humidity
  - **Screen 2 (5s):** Pressure & Altitude
  - **Screen 3 (5s):** Gas Resistance, IAQ, Accuracy, Air Quality Status
  - **Screen 4 (5s):** System Uptime (HH:MM:SS)
- **Incremental Refresh:** Screen only updates when values change beyond thresholds, reducing flicker and CPU usage.
- **Battery Optimized:** Sensor reads every 30 seconds with intelligent power management for 24hr+ battery life.

### Overheat Protection

- If temperature ≥ **45°C**, device enters "HOT HOLD" mode.
- OLED shows a warning with a thermometer icon and the text **Overheat**, which shifts position every 60s to prevent burn-in.
- Normal operation resumes when temperature drops below **41°C**.

### Robustness

- **Error Recovery:** Auto-retry mechanism for BSEC initialization with exponential backoff.
- **Offline Operation:** Continues to work without network, using last known QNH or default fallback.
- **Serial Command Interface:** Configure QNH, WiFi, coordinates, check status, and calibrate altitude via serial monitor.

### Periodic WiFi QNH Sync (battery-friendly)

- Radio stays **OFF**; wakes **hourly** to fetch `current.pressure_msl` from Open-Meteo, then sleeps again.
- Each window tries up to **3× within ~1 minute** (15s timeout each); if all fail, radio sleeps and retries in 15 minutes.
- No WiFi credentials → pure offline mode with EEPROM default QNH (existing behavior).
- WiFi drop mid-sync → keeps last valid QNH, retries in 15 minutes.
- ⚠️ ESP8266 is **2.4 GHz only** — a 5 GHz-only AP can never connect. Diagnose via serial: `st=1` = AP not visible (wrong SSID / 5 GHz / out of range), `st=4` = auth failed (wrong password), `st=6` = disconnected mid-way.
- **Manual `QNH=`/`ALTREF=` always wins** and freezes auto-sync (re-enable with `QNHMODE=AUTO`) so field calibration is never silently overridden.
- Auto-apply uses a **0.4 hPa deadband** (~3 m) to ignore API jitter and keep altitude stable.
- Every successful fetch marks source **AUTO** (persisted across reboots), even when the value is held by the deadband — so the display always tells the truth about data freshness.
- Screen 4 (Uptime) shows sync state: `AUTO 35m` (synced, age) / `WIFI WAIT` (creds ready, first sync pending) / `SYNC FAIL` (attempts failing, check serial log) / `MANUAL` / `QNH DEF` (no creds).

---

## Hardware Components

- **ESP8266 Board with Integrated OLED (0.96" SSD1306).**
- **Bosch BME680 Sensor** (I2C).
- **TP4056** for LiPo battery charging and management.
- **LiPo Battery** (e.g., 1500–2100 mAh).
- **Power Switch** for on/off control.
- **Custom PCB & Enclosure** for portability.

---

## Pin Configuration

| Component          | Pin / Address |
| ------------------ | ------------- |
| I2C SDA (Data)     | `GPIO 14`     |
| I2C SCL (Clock)    | `GPIO 12`     |
| OLED I2C Address   | `0x3C`        |
| BME680 I2C Address | `0x76`        |

---

## Software Requirements

- Arduino IDE
- ESP8266 Board Package for Arduino IDE
- Libraries:
  - `Wire.h` (Built-in)
  - `EEPROM.h` (Built-in)
  - `Adafruit GFX Library`
  - `Adafruit SSD1306`
  - `Bosch BSEC Software Library`

---

## ⚠️ Important: IRAM Overflow Fix

If you encounter this compilation error:

```
section `.text1' will not fit in region `iram1_0_seg'
```

This is a **memory configuration issue**, not a code problem. The fix requires two settings changes:

### Quick Fix (Arduino IDE)

1. **Tools** → **Flash Size** → Change to `4M (3M SPIFFS)`
2. **Tools** → **MMU** → Select `16KB cache + 48KB IRAM`
3. **Restart Arduino IDE**

### Complete Guide

See **[FIX_IRAM_OVERFLOW_GUIDE.md](FIX_IRAM_OVERFLOW_GUIDE.md)** for:

- Detailed step-by-step instructions
- VS Code / PlatformIO configuration
- Manual `platform.txt` replacement method
- Troubleshooting and technical explanations

**Included in this project:** `platform.txt` (ready-to-use replacement file)

---

## Installation & Setup

1. **Install Arduino IDE & ESP8266 Core:**
   Add `http://arduino.esp8266.com/stable/package_esp8266com_index.json` to Board Manager URLs.

2. **Install Required Libraries** via Library Manager:
   - "Adafruit GFX Library"
   - "Adafruit SSD1306"
   - "BSEC Software Library"

3. **Configure the Sketch:**

   ```cpp
   // Edit configuration constants in config.h
   #define SENSOR_READ_INTERVAL_MS 30000UL    // Sensor reading interval
   #define OLED_DATA_SCREEN_1_DURATION 5000UL // Screen 1 duration
   #define OLED_DATA_SCREEN_2_DURATION 5000UL // Screen 2 duration
   #define OLED_DATA_SCREEN_3_DURATION 5000UL // Screen 3 duration
   #define OLED_DATA_SCREEN_4_DURATION 5000UL // Screen 4 (Uptime) duration
   ```

4. **Upload to ESP8266:**
   - Go to `Tools > Board` and select "NodeMCU 1.0 (ESP-12E Module)"
   - **Tools > Flash Size** → `4M (3M SPIFFS)`
   - **Tools > MMU** → `16KB cache + 48KB IRAM`
   - Connect your board and select the correct COM port
   - Click "Upload"

---

## Usage in the Field

### 🔹 WiFi Provisioning (for hourly auto-QNH)

1. Connect via serial (115200 baud) and type:
   ```
   WIFI_SSID=YourSSID
   WIFI_PASS=YourPassword
   LAT=-6.898284
   LON=107.634983
   ```
2. Verify with `WIFI?` and force a test sync with `SYNCNOW`.
3. The radio stays OFF except for a ~20s window each hour. First sync runs ~30s after boot.
4. Credentials are stored in EEPROM (plaintext — anyone with physical access can read them). Never commit `src/communication/wifi_secrets.h`.

> **Note:** Manual `QNH=`/`ALTREF=` disables auto-sync to protect field calibration. Re-enable with `QNHMODE=AUTO`.

### 🔹 Calibration via QNH (when local QNH data is available)

1. Connect your Android phone to the device via **USB-C OTG + data cable**.
2. Open **Serial USB Terminal** (or any compatible terminal app).
3. Set baud rate to **115200**, newline to **CR+LF**.
4. Type:
   ```
   QNH=1013.25
   ```
   (Adjust this to the latest QNH in your location. Check weather apps such as [Breezy Weather](https://github.com/breezy-weather/breezy-weather) for current atmospheric pressure)

The device stores QNH in EEPROM and immediately recalculates altitude.

### 🔹 Calibration via ALTREF (when you know the exact elevation)

1. Stand at a location with a verified elevation (e.g., summit, basecamp, map marker).
2. Type:
   ```
   ALTREF=709
   ```
   (Adjust this according to the altitude of your location)

The device computes QNH from the current pressure and saves it permanently. Verify with:

```
ALT?
```

### 🔹 Hiking / Trekking Scenario

- At **basecamp**, perform one calibration (QNH **or** ALTREF).
- Recalibrate if the weather changes significantly.
- **Precision Rule:** ±**1 hPa** ≈ ±**8–9 m** at mid-altitudes. Recalibrate whenever you change location or weather conditions vary.

---

## How It Works

### IAQ (Indoor Air Quality) Index

The BSEC library provides an IAQ value on a scale from 0 to 500:

- **0 - 50:** Excellent
- **51 - 100:** Good
- **101 - 150:** Lightly Polluted
- **151 - 200:** Moderately Polluted
- **201 - 300:** Heavily Polluted
- **301+:** Severely Polluted

The device displays this scale as an "AQS" (Air Quality Status) string for easy interpretation. The `Acc` (Accuracy) value indicates the BSEC algorithm's confidence level (0=stabilizing, 1=low, 2=medium, 3=high). High accuracy is typically achieved after the device runs for a while.

### Serial Commands

Connect via serial monitor (115200 baud) and use:

| Command          | Description                            | Example Response                    |
| ------------------ | -------------------------------------- | ----------------------------------- |
| `QNH=<hPa>`        | Set sea-level pressure (MAN, auto off) | `OK QNH=1013.25 hPa (MAN, auto off)`|
| `QNH?`             | Query current QNH value                | `QNH=1013.25 hPa`                   |
| `ALT?`             | Query current altitude                 | `ALT=708.9 m`                       |
| `PRESS?`           | Query current pressure                 | `P=933.10 hPa`                      |
| `ALTREF=<m>`       | Auto-calculate QNH from known altitude | `OK QNH=1013.25 hPa (MAN, auto off)`|
| `STATUS`           | Show system status                     | `Mode:0 Therm:0 BSEC:Y IAQ:45.2(3)` |
| `WIFI_SSID=<s>`    | Save WiFi SSID (1..32 chars)           | `OK WIFI_SSID=Hackerman`            |
| `WIFI_PASS=<p>`    | Save WiFi password (never echoed)      | `OK WIFI saved`                     |
| `WIFI?`            | Show WiFi/QNH-sync status              | `WIFI ssid=... auto=Y src=AUTO ...` |
| `WIFICLEAR`        | Erase stored WiFi credentials          | `OK WIFI cleared`                   |
| `LAT=<f>` / `LON=<f>` | Set Open-Meteo coordinates          | `OK LAT=-6.898284`                  |
| `QNHMODE=AUTO\|MAN`| Enable/disable auto QNH sync           | `OK QNHMODE=AUTO`                   |
| `SYNCNOW`          | Force an immediate sync attempt        | `OK sync scheduled`                 |
| `HELP`             | Show all commands                      | `CMD: QNH=<hPa>\|...`               |

### Display Screens

The OLED continuously cycles through 4 screens:

1. **Screen 1 (5s):** Temperature & Humidity
2. **Screen 2 (5s):** Pressure & Altitude
3. **Screen 3 (5s):** Gas Resistance, IAQ, Accuracy, AQS
4. **Screen 4 (5s):** System Uptime (HH:MM:SS) + live `QNH xxxx.x` value + sync age line (`SYNC 10m AGO` / `WIFI WAIT` / `SYNC FAIL` / `MANUAL` / `QNH DEF`)
5. **Loops back to Screen 1** → Continuous monitoring mode

### IAQ Scale

| IAQ Range | Air Quality |
| --------- | ----------- |
| 0-50      | Excellent   |
| 51-100    | Good        |
| 101-150   | Light       |
| 151-200   | Moderate    |
| 201-300   | Unhealthy   |
| 301+      | Hazardous   |

### IAQ Accuracy

The `Acc` value (0-3) shows BSEC algorithm confidence:

- **0:** Stabilizing (first 5-30 minutes)
- **1:** Low confidence
- **2:** Medium confidence
- **3:** High confidence (optimal)

---

## Power Management

### Display Behavior

- **Continuous Mode:** Display never turns off
- **4 screens cycle** every 5 seconds each (20s total cycle)
- **Conditional redraw:** Only updates when values change beyond thresholds (reduces flicker)

### Battery Life Estimates

| Battery Capacity | Estimated Runtime |
| ---------------- | ----------------- |
| 1500 mAh         | ~24-30 hours      |
| 2000 mAh         | ~30-36 hours      |
| 2100 mAh         | ~32-40 hours      |

_Estimates based on continuous display mode with 30s sensor intervals_

---

## Advanced Features

### Thermal Protection

When temperature reaches **45°C**:

- Display shows animated "Overheat" warning
- Contrast reduced to minimum (prevents OLED damage)
- Warning icon position changes every 60s (prevents burn-in)
- Normal operation resumes at **41°C**

### Transport Detection

The device detects movement via rapid environmental changes:

- Freezes gas baseline calibration during transport
- Resumes calibration after 6 stable readings
- Prevents false VOC readings during relocation

### Adaptive IAQ Smoothing

IAQ display uses variance-adaptive filtering:

- **Low variance** (stable): Heavy smoothing (α=0.10) for stable reading
- **High variance** (changing): Light smoothing (α=0.45) for responsiveness
- **Accuracy-based:** Reduces smoothing when accuracy is low

---

## Accuracy Notes

- **Formula:** Based on the standard barometric equation. Real-world temperature deviations may cause small altitude offsets.
- **Sensor Tolerance:** BME680 pressure bias ≈ ±1 hPa → ±8–9 m error.
- **Weather Impact:** QNH drifts naturally with weather; recalibrate as needed during extended outdoor sessions.

---

## Security Notes

- **WiFi fetch uses plaintext HTTP** (no TLS): deliberate tradeoff — ESP8266 IRAM is at ~97% with BSEC + display, TLS would not fit reliably. `pressure_msl` is public weather data; a MITM can at worst feed a wrong QNH, which is contained by the 870–1100 hPa validator + 0.4 hPa deadband.
- **Credentials at rest are plaintext** (EEPROM + local `wifi_secrets.h`, gitignored). Anyone with physical USB access can already reflash the device, so the threat model assumes trusted hands. Run `WIFICLEAR` before lending the unit.
- **No listening services**: no AP mode, no mDNS, no open ports. The only inputs are USB serial (physical) and the hourly Open-Meteo response (validated before use).
- **WiFi password is never echoed** on serial (`WIFI_PASS` replies `OK WIFI saved` only).

## Troubleshooting

### ⚠️ IRAM Overflow Error (Compilation Error)

**Symptoms:**

```
section `.text1' will not fit in region `iram1_0_seg'
Compilation error: exit status 1
```

**Solution:** This is a memory configuration issue, NOT a code problem.

**Quick Fix:**

1. **Arduino IDE:**
   - Tools → Flash Size → Change to `4M (3M SPIFFS)`
   - Tools → MMU → Select `16KB cache + 48KB IRAM`
   - Restart Arduino IDE
2. **VS Code:**
   - Edit `.vscode/arduino.json`
   - Change `eesz=1M64` → `eesz=4M3M`
   - Change `mmu=3232` → `mmu=4816`
   - Reload window

**Complete Guide:** See [`FIX_IRAM_OVERFLOW_GUIDE.md`](FIX_IRAM_OVERFLOW_GUIDE.md) for detailed instructions and manual `platform.txt` replacement method.

### BSEC Initialization Fails

**Symptoms:** "Sensor Error" screen, "Retrying..."

**Solutions:**

1. Check I2C wiring (GPIO14=SDA, GPIO12=SCL)
2. Verify BME680 address (0x76 or 0x77)
3. Check serial monitor for error messages
4. Power cycle device

### IAQ Accuracy Stays at 0-1

**Cause:** BSEC needs time to stabilize

**Solution:** Leave device running for 30 minutes - 24 hours for full calibration

### Display Flickers

**Cause:** Normal when values change frequently

**Solution:** Adjust thresholds (`TH_T`, `TH_H`, etc.) to reduce update frequency

### EEPROM Corruption

**Symptoms:** Settings reset on reboot

**Solution:** Device auto-detects and repairs corrupted EEPROM

---

## Code Structure

```
ESP8266Display-BME680.ino   (setup + main loop)
└── src/
    ├── config.h             (all #defines, memory map)
    ├── types.h              (enumerations)
    ├── globals.h / .cpp     (global state variables)
    ├── utils.h / .cpp       (utility functions: baro, IAQ categories)
    ├── display/
    │   └── oled.h / .cpp    (OLED display + state machine)
    ├── sensing/
    │   ├── sensor.h / .cpp  (BSEC/BME680, thermal, error recovery)
    │   └── storage.h / .cpp (EEPROM persistence)
    └── communication/
        ├── serial_cmd.h / .cpp (serial parser, char buffer)
        ├── wifi_qnh.h / .cpp   (periodic WiFi + Open-Meteo QNH sync)
        ├── wifi_secrets.h      (local creds, gitignored - DO NOT COMMIT)
        └── wifi_secrets.example.h (template)
```

---

## Project Files

| File                                   | Description                                       |
| -------------------------------------- | ------------------------------------------------- |
| `ESP8266Display-BME680.ino`            | Main entry point (setup + loop)                   |
| `src/config.h`                         | Hardware & algorithm configuration constants      |
| `src/types.h`                          | Enum type definitions                             |
| `src/globals.h` / `src/globals.cpp`    | Global state variables                            |
| `src/utils.h` / `src/utils.cpp`        | Utility functions (barometric, math, categories)  |
| `src/display/oled.h` / `src/display/oled.cpp` | OLED display & screen state machine        |
| `src/sensing/sensor.h` / `src/sensing/sensor.cpp` | BSEC/BME680 driver, filtering, thermal, retry |
| `src/sensing/storage.h` / `src/sensing/storage.cpp` | EEPROM persistence layer                  |
| `src/communication/serial_cmd.h` / `src/communication/serial_cmd.cpp` | Serial parser (char buffer) |
| `src/communication/wifi_qnh.h` / `src/communication/wifi_qnh.cpp` | Periodic WiFi + Open-Meteo sync |
| `src/communication/wifi_secrets.h` | Local WiFi defaults (gitignored) |
| `ESP8266Display-BME680.ino.backup`     | Original v1.x backup                              |
| `platform.txt`                     | Fixed platform configuration for IRAM fix       |
| `FIX_IRAM_OVERFLOW_GUIDE.md`       | Complete IRAM overflow fix guide                |
| `Fix RAM ESP8266.txt`              | Original fix notes                              |
| `README.md`                        | This file - project documentation               |
| `CHANGELOG.md`                     | Version history and changes                     |
| `MIGRATION.md`                     | Upgrade guide from v1.x to v2.0                 |

---

## Performance Metrics

### Memory Usage (Typical)

- **Flash:** ~380 KB (varies with libraries)
- **SRAM:** ~42 KB
- **EEPROM:** 512 bytes (128 bytes used)

### Timing

- **Boot Time:** ~2 seconds to first reading
- **BSEC Stabilization:** 5-30 minutes
- **Sensor Read Cycle:** 30 seconds
- **Display Cycle:** 20 seconds (4 screens, continuous loop)
