# High-Precision ESP8266 BME680 Environmental Monitor

A high-precision, portable environmental monitor using an ESP8266, a Bosch BME680 sensor, and an integrated OLED display. This project leverages the official **Bosch BSEC (Bosch Sensortec Environmental Cluster) software library** to provide accurate Indoor Air Quality (IAQ) readings, alongside temperature, humidity, pressure, and altitude.

The device is designed for continuous operation with robust error handling, intelligent baseline management, and an **always-on OLED display** that switches between screens every few seconds. It also supports periodic fetching of QNH (sea-level pressure) data from the internet to enhance altitude accuracy.

[![Last Updated](https://img.shields.io/github/last-commit/mysteriza/ESP8266Display-BME680?label=Last%20Updated)](https://github.com/mysteriza/ESP8266Display-BME680/commits/main)

<img src="https://github.com/user-attachments/assets/dc004bd2-9f50-4d7b-9746-8caaaf38583f" alt="ESP8266 Device Front View" width="400">
<img src="https://github.com/user-attachments/assets/871008fa-13bc-48c8-884a-2664ee2e2b26" alt="ESP8266 Device Side View" width="400">
<img src="https://github.com/user-attachments/assets/62e0e651-10e9-4774-bc52-773508a8fb07" alt="ESP8266 Device Side View" width="400">

---

## Key Features

### Core Sensing & IAQ
* **Bosch BSEC Integration:** Utilizes Bosch's proprietary BSEC algorithm for reliable **Indoor Air Quality (IAQ)** index calculation (0–500 scale).
* **Comprehensive Data:** Measures Temperature, Humidity, Barometric Pressure, Gas Resistance (VOCs), and calculates Altitude.
* **Data Persistence:** Saves the BSEC calibration state to EEPROM every **4 hours**, but only when the IAQ accuracy level is high (`iaqAccuracy==3`). This minimizes flash wear and ensures stable calibration across power cycles.

### Advanced Data Processing
* **Variance-Aware IAQ Smoothing:** Implements an adaptive smoothing algorithm on the Static IAQ value, providing a more stable and human-readable output.
* **Self-Adapting Gas Baseline:** Automatically adapts over time for long-term environmental stability.
* **Transport-Aware Logic:** Freezes baseline calibration when rapid environmental changes are detected (e.g., device movement).
* **Altitude Filtering:** Combines a median filter with an EMA for smooth and reliable altitude readings.

### Connectivity & Accuracy
* **Automatic QNH Updates:** Connects to Wi-Fi every **1 hour** to fetch the current sea-level pressure (QNH) from the Open-Meteo API.  
* **Scientific Altitude Formula:** Accounts for temperature and humidity for improved altitude accuracy.

### Display & Power
* **Always-On Display (AOD):** OLED display remains active continuously, cycling through two screens:
  * **Screen 1 (5s):** Temperature, Humidity, Pressure, Altitude.  
  * **Screen 2 (5s):** Gas Resistance, Static IAQ, IAQ Accuracy, and Air Quality Status (AQS).  
* **Incremental Refresh:** Screen only updates when values change beyond thresholds, reducing flicker and improving efficiency.
* **Overheat Protection:**  
  * If temperature ≥ **45°C**, device enters "HOT HOLD" mode.  
  * OLED shows a warning with a thermometer icon and the text **Overheat**, which shifts position every 60s to prevent burn-in.  
  * Sensor reads less frequently (every 60s) until the temperature drops below **41°C**, at which point normal operation resumes.

### Robustness
* **Error Recovery:** Auto-retry mechanism for BSEC initialization with exponential backoff.
* **Watchdog Handling:** Automatically restarts if stalled.
* **Offline Operation:** Continues to work without Wi-Fi, using last known QNH or default fallback.

---

## Hardware Components
* **ESP8266 Board with Integrated OLED (0.96" SSD1306).**
* **Bosch BME680 Sensor** (I2C).
* **TP4056** for LiPo battery charging and management.
* **LiPo Battery** (e.g., 1500–2100 mAh).
* **Power Switch** for on/off control.
* **Custom PCB & Enclosure** for portability.

---

## Pin Configuration
* **I2C SDA (Data):** `GPIO 14`  
* **I2C SCL (Clock):** `GPIO 12`  
* **OLED I2C Address:** `0x3C`  
* **BME680 I2C Address:** `0x76`  

---

## Software Requirements
* Arduino IDE  
* ESP8266 Board Package for Arduino IDE  
* Libraries:  
  * `Wire.h` (Built-in)  
  * `EEPROM.h` (Built-in)  
  * `Adafruit GFX Library`  
  * `Adafruit SSD1306`  
  * `Bosch BSEC Software Library`  

---

## Installation & Setup
1. **Install Arduino IDE & ESP8266 Core:**  
   Add `http://arduino.esp8266.com/stable/package_esp8266com_index.json` to Board Manager URLs.  
2. **Install Required Libraries** via Library Manager:  
   * "Adafruit GFX Library"  
   * "Adafruit SSD1306"  
   * "BSEC Software Library"  
3. **Configure the Sketch:**  
   ```cpp
   const char* WIFI_SSID = "YourWiFi_SSID";
   const char* WIFI_PASS = "YourWiFi_Password";
   const float OM_LAT = -6.914744f; // Change to your latitude
   const float OM_LON = 107.609810f; // Change to your longitude
   ```
4.  **Upload to ESP8266:**
    * Go to `Tools > Board` and select your specific ESP8266 board (e.g., "NodeMCU 1.0 (ESP-12E Module)").
    * Connect your board and select the correct COM port.
    * Click "Upload".

## How It Works

### IAQ (Indoor Air Quality) Index
The BSEC library provides an IAQ value on a scale from 0 to 500:
* **0 - 50:** Excellent
* **51 - 100:** Good
* **101 - 150:** Lightly Polluted
* **151 - 200:** Moderately Polluted
* **201 - 300:** Heavily Polluted
* **301+:** Severely Polluted

The device displays this scale as an "AQS" (Air Quality Status) string for easy interpretation. The `Acc` (Accuracy) value indicates the BSEC algorithm's confidence level (0=stabilizing, 1=low, 2=medium, 3=high). High accuracy is typically achieved after the device runs for a while.
