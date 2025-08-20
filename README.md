# High-Precision ESP8266 BME680 Environmental Monitor

A high-precision, portable environmental monitor using an ESP8266, a Bosch BME680 sensor, and an integrated OLED display. This project leverages the official **Bosch BSEC (Bosch Sensortec Environmental Cluster) software library** to provide accurate Indoor Air Quality (IAQ) readings, alongside temperature, humidity, pressure, and altitude.

The device is designed for intelligent, low-power operation and features advanced algorithms for data smoothing, baseline self-calibration, and robust error handling. While it can operate fully offline, it enhances altitude accuracy by periodically fetching local sea-level pressure (QNH) data from the internet.

[![Last Updated](https://img.shields.io/github/last-commit/mysteriza/ESP8266Display-BME680?label=Last%20Updated)](https://github.com/mysteriza/ESP8266Display-BME680/commits/main)

<img src="https://github.com/user-attachments/assets/dc004bd2-9f50-4d7b-9746-8caaaf38583f" alt="ESP8266 Device Front View" width="400">
<img src="https://github.com/user-attachments/assets/871008fa-13bc-48c8-884a-2664ee2e2b26" alt="ESP8266 Device Side View" width="400">
<img src="https://github.com/user-attachments/assets/62e0e651-10e9-4774-bc52-773508a8fb07" alt="ESP8266 Device Side View" width="400">

## Key Features

### Core Sensing & IAQ
* **Bosch BSEC Integration:** Utilizes Bosch's proprietary BSEC algorithm for reliable **Indoor Air Quality (IAQ)** index calculation (0-500 scale).
* **Comprehensive Data:** Measures Temperature, Humidity, Barometric Pressure, Gas Resistance (VOCs), and calculates Altitude.
* **Data Persistence:** Saves the BSEC calibration state to EEPROM, allowing the sensor to maintain its accuracy across power cycles without needing a lengthy re-calibration period.

### Advanced Data Processing
* **Variance-Aware IAQ Smoothing:** Implements an adaptive smoothing algorithm on the Static IAQ value, providing a more stable and human-readable output that filters out momentary noise while still reacting to significant environmental changes.
* **Self-Adapting Gas Baseline:** The gas resistance baseline automatically and slowly adapts to the long-term environment, ensuring IAQ readings remain relevant over time.
* **Transport-Aware Logic:** Intelligently freezes baseline calibration when rapid changes in pressure, humidity, or IAQ are detected (e.g., moving the device between rooms or outdoors), preventing incorrect calibration drift.
* **Altitude Filtering:** Uses a median filter and an Exponential Moving Average (EMA) to provide a smooth and reliable altitude reading, rejecting spurious outliers.

### Connectivity & Accuracy
* **Automatic QNH Updates:** Optionally connects to Wi-Fi on boot and periodically (once per hour) to fetch the current sea-level pressure (QNH) from the Open-Meteo API for the specified latitude/longitude. This dramatically improves the accuracy of altitude calculations.
* **Scientific Altitude Formula:** Employs a more precise altitude formula that accounts for temperature and humidity, providing better results than the standard barometric formula alone.

### Hardware & Power Management
* **OLED Display Cycle:**
    * **Screen 1 (5s):** Temperature, Humidity, Pressure, Altitude.
    * **Screen 2 (5s):** Gas Resistance, Static IAQ, IAQ Accuracy, and Air Quality Status (AQS).
    * **Power Save (20s):** The display turns off to conserve energy.
* **Efficient Operation:** The main sensor reading loop runs every 30 seconds. During the 20-second display-off phase, the device enters a light sleep mode (disabling Wi-Fi) to minimize power consumption while continuing to process sensor data.
* **Robustness:** Includes an auto-retry mechanism with exponential backoff for sensor initialization and a watchdog to automatically restart the device if it stalls.

## Hardware Components
* **ESP8266 Board with Integrated OLED:** with a built-in 0.96" SSD1306 OLED.
* **Bosch BME680 Sensor:** The core environmental sensor connected via I2C.
* **TP4056:** For LiPo battery charging and management.
* **LiPo Battery:** 1500 mAh for portable use.
* **Power Switch:** For turning the device on and off.
* **Custom PCB & Enclosure:** To house the components neatly.

## Pin Configuration
The project uses the following default I2C pins for both the OLED display and the BME680 sensor:
* **I2C SDA (Data):** `GPIO 14`
* **I2C SCL (Clock):** `GPIO 12`
* **OLED I2C Address:** `0x3C`
* **BME680 I2C Address:** `0x76`

**Note:** On most ESP8266 boards with an integrated OLED, these pins are pre-wired.

## Software Requirements
* Arduino IDE
* ESP8266 Board Package for Arduino IDE
* **Libraries:**
    * `Wire.h` (Built-in)
    * `EEPROM.h` (Built-in)
    * `Adafruit GFX Library` (by Adafruit)
    * `Adafruit SSD1306` (by Adafruit)
    * `Bosch BSEC Software Library` (by Bosch Sensortec)

## Installation & Setup
1.  **Install Arduino IDE & ESP8266 Core:**
    * Install the latest [Arduino IDE](https://www.arduino.cc/en/software).
    * In `File > Preferences`, add `http://arduino.esp8266.com/stable/package_esp8266com_index.json` to "Additional Board Manager URLs".
    * Go to `Tools > Board > Boards Manager...`, search for "esp8266", and install the package.
2.  **Install Adafruit Libraries:**
    * In Arduino IDE, go to `Sketch > Include Library > Manage Libraries...`.
    * Search for and install:
        * "Adafruit GFX Library"
        * "Adafruit SSD1306"
3.  **Install Bosch BSEC Library:**
    * Download the latest release of the BSEC library from the [official Bosch Sensortec GitHub](https://github.com/boschsensortec/BSEC-Arduino-library).
    * In the Arduino IDE, go to `Sketch > Include Library > Add .ZIP Library...` and select the downloaded ZIP file.
    * **Important:** You must also agree to the license agreement provided by Bosch. Follow the on-screen instructions or documentation from Bosch.
4.  **Configure the Sketch:**
    * Open the `.ino` file in the Arduino IDE.
    * **Modify Wi-Fi credentials and location** for the QNH feature:
        ```cpp
        const char* WIFI_SSID = "YourWiFi_SSID";
        const char* WIFI_PASS = "YourWiFi_Password";
        const float OM_LAT = -6.914744f; // Your Latitude
        const float OM_LON = 107.609810f; // Your Longitude
        ```
5.  **Upload to ESP8266:**
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

### Serial Monitor Commands
Connect to the device using the Arduino Serial Monitor at a baud rate of **115200**.
* `getbaseline`: Prints the current gas resistance baseline value and its readiness status.
* `resetbaseline`: Resets the stored gas baseline, forcing the device to start a new calibration cycle. Useful when moving the device to a completely new long-term environment.
