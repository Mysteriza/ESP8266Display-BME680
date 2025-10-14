# High-Precision ESP8266 BME680 Environmental Monitor

A portable, high-precision environmental monitor built with **ESP8266 + BME680 + OLED**, leveraging the **Bosch BSEC (Bosch Sensortec Environmental Cluster)** library for stable **Indoor Air Quality (IAQ)** readings, as well as temperature, humidity, pressure, and altitude.  
The latest firmware introduces **barometric calibration via Serial**, allowing you to set **QNH** (sea-level pressure) directly or **ALTREF** (known reference altitude) for location- and time-accurate altitude measurement.  
The QNH value is **stored persistently in EEPROM** and remains active across power cycles.

[![Last Updated](https://img.shields.io/github/last-commit/mysteriza/ESP8266Display-BME680?label=Last%20Updated)](https://github.com/mysteriza/ESP8266Display-BME680/commits/main)

<img src="https://github.com/user-attachments/assets/16ce6f7f-89be-46ac-ae11-b34ff67bfa96" alt="ESP8266 Device Front View" width="400">

<img src="https://github.com/user-attachments/assets/261b5c6e-4504-447e-8210-7e87774dcaeb" alt="ESP8266 Device Side View" width="400">

<img src="https://github.com/user-attachments/assets/99518c65-6359-4fd6-8f37-d91e0dde18bd" alt="ESP8266 Device Rear View" width="400">

<img src="https://github.com/user-attachments/assets/62e0e651-10e9-4774-bc52-773508a8fb07" alt="ESP8266 Device Rear View" width="400">

---

## Key Features

### Core Sensing & IAQ
- **Bosch BSEC Integration:** Provides an IAQ index (0–500) with accuracy levels (`Acc 0–3`).
- **Comprehensive Data:** Measures temperature, humidity, **pressure** (hPa), **gas/VOC** (kΩ), and computes **altitude** (m).
- **Persistent Calibration:** Saves BSEC state to EEPROM every **4 hours** (only when `iaqAccuracy==3`) to minimize flash wear and preserve calibration across reboots.

### Advanced Data Processing
- **Variance-Aware IAQ Smoothing:** Adaptive smoothing for static IAQ—stable yet responsive.
- **Self-Adapting Gas Baseline:** Automatically adjusts over time; freezes baseline when rapid environmental changes are detected.
- **Altitude Filtering:** Median-of-3 → outlier clamp (±18 m) → **EMA** with **0.4 m deadband** for smooth and stable altitude readings.

### Calibration & Accuracy
- **Serial-Based Calibration (No Wi-Fi Needed):**
  - `QNH=<hPa>` → set local QNH, store to **EEPROM**, persist across reboots.
  - `ALTREF=<m>` → compute QNH from known **altitude reference**, save and apply automatically.
  - `QNH?`, `ALT?`, `PRESS?` → quick inspection commands.
- **Automatic Filter Reset:** Setting QNH/ALTREF resets altitude smoothing for instant convergence.
- **Precision Rule:** ±**1 hPa** ≈ ±**8–9 m** at mid-altitudes.  
  Recalibrate whenever you change location or weather conditions vary.

### Display & Power
- **Automatic OLED Rotation:** Three screens cycle every 5 seconds with change-detection refresh:
  - **Screen 1:** Temperature, Humidity  
  - **Screen 2:** Pressure, Altitude  
  - **Screen 3:** Gas (kΩ), Static IAQ, Accuracy, AQS (Air Quality Status)  
- **Overheat Protection:**  
  - Enters **HOT HOLD** mode at ≥45 °C with a moving “Overheat” warning.  
  - Resumes normal operation when ≤41 °C.

### Robustness
- **BSEC Auto-Retry:** Exponential backoff recovery if initialization fails.  
- **Offline-First Operation:** Works fully offline using the last saved QNH.  

---

## Hardware Components
- **ESP8266 Dev Board** + **0.96" SSD1306 OLED (I2C)**  
- **Bosch BME680 Sensor** (I2C)  
- **LiPo + TP4056** (optional, for portable setups)  
- Power switch, custom PCB/enclosure (optional)

### Pin Configuration
| Function | Pin / Address |
|-----------|----------------|
| I2C SDA | `GPIO 14` |
| I2C SCL | `GPIO 12` |
| OLED Address | `0x3C` |
| BME680 Address | `0x76` |

---

## Usage in the Field

### 🔹 Calibration via QNH (when local QNH data is available)
1. Connect your Android phone to the device via **USB-C OTG + data cable**.  
2. Open **Serial USB Terminal** (or any compatible terminal app).  
3. Set baud rate to **115200**, newline to **CR+LF**.  
4. Type:
   - ```QNH=1013.25``` (Adjust this to the latest QNH in your location. Please check weather apps such as [Breezy Weather](https://github.com/breezy-weather/breezy-weather) and similar apps to see the atmospheric pressure in your area)

The device stores QNH in EEPROM and immediately recalculates altitude.

---

### 🔹 Calibration via ALTREF (when you know the exact elevation)
1. Stand at a location with a verified elevation (e.g., summit, basecamp, map marker).  
2. Type:
```ALTREF=709``` (Adjust this according to the altitude of your location)

The device computes QNH from the current pressure and saves it permanently.  
Verify with the command below to see the current altitude:
```ALT?```

---

### 🔹 Hiking / Trekking Scenario
- At **basecamp**, perform one calibration (QNH **or** ALTREF).  
- Recalibrate if the weather changes significantly. 

---

## Serial Command Reference

| Command | Function | Example Response |
|----------|-----------|------------------|
| `QNH=1012.8` | Set QNH (hPa), save to EEPROM, reset altitude filter | `OK QNH=1012.80 hPa` |
| `QNH?` | Show current QNH | `QNH=1012.80 hPa` |
| `ALTREF=709` | Compute QNH from 709 m reference and save | `OK QNH from ALTREF -> 1014.32 hPa` |
| `ALT?` | Show current altitude (m) | `ALT=708.9 m` |
| `PRESS?` | Show current pressure (hPa) | `P=933.10 hPa` |
| `HELP` | List all available commands | `CMD: QNH=... | QNH? | ALT? | PRESS? | ALTREF=...` |

---

## Accuracy Notes
- **Formula:** Based on the standard barometric equation. Real-world temperature deviations may cause small altitude offsets.  
- **Sensor Tolerance:** BME680 pressure bias ≈ ±1 hPa → ±8–9 m error.  
- **Weather Impact:** QNH drifts naturally with weather; recalibrate as needed during extended outdoor sessions.  
