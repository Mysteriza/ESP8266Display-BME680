# ESP8266 BME680 Environmental Monitor with OLED Display

This project provides a portable, offline environmental monitoring solution using an ESP8266 microcontroller with a built-in OLED display and a BME680 gas and environmental sensor.

<img src="https://github.com/user-attachments/assets/a5e39dbc-0e6a-4b88-b850-ccc1f0e625ef" alt="ESP8266 Device Front View" width="400">
<img src="https://github.com/user-attachments/assets/25655a07-ef23-4456-b626-bd6670e3cddc" alt="ESP8266 Device Side View" width="400">

## Features

* **Environmental Sensing:** Reads Temperature, Humidity, Pressure, Altitude, and Gas Resistance using the Bosch BME680 sensor.
* **Offline Operation:** All data processing and display occur on the device, requiring no external internet connection for core functionality.
* **OLED Display:**
    * **Screen 1:** Displays Temperature, Humidity, Pressure, and Altitude.
    * **Screen 2:** Displays Gas Resistance (kOhm) and an interpreted Gas Status (e.g., "Very Good", "Good", "Bad").
    * **Power Saving:** The OLED display cycles between Screen 1 (7 seconds) and Screen 2 (3 seconds), then turns off for the remainder of the 30-second sensor reading interval (approximately 20 seconds) to conserve battery.
* **Efficient Sensor Readings:** The BME680 sensor is put into a low-power state between readings and only actively performs measurements every 30 seconds to save power.
* **Robustness:** Includes retry mechanisms for BME680 initialization and data reading to handle temporary communication issues.

## Hardware Used

* **ESP8266 Development Board:** Specifically designed for ESP8266 boards with built-in OLED displays (e.g., NodeMCU ESP8266 with integrated 0.96" SSD1306 OLED).
* **Bosch BME680 Environmental Sensor:** Connected via I2C.
* **TP4506:** For LiPo battery charging/management
* **LiPo Battery 1500 mAh:** For portable operation.
* **Power Switch:** For device control.
* **Custom PCB & Enclosure:** For integration and portability.

## Pin Configuration

The project uses the following default I2C pins for the OLED display and BME680 sensor:

* **I2C SDA (Data):** GPIO 14
* **I2C SCL (Clock):** GPIO 12
* **OLED I2C Address:** 0x3C (common for 0.96" SSD1306)
* **BME680 I2C Address:** 0x76

**Note:** For ESP8266 boards with built-in OLED, these pins are often pre-wired internally. Verify your specific board's pinout if you encounter issues.

## Software Requirements

* Arduino IDE
* ESP8266 Board Package for Arduino IDE
* **Libraries:**
    * `Wire.h` (Built-in Arduino)
    * `Adafruit_Sensor` (by Adafruit)
    * `Adafruit_BME680` (by Adafruit)
    * `SSD1306Wire` (by Daniel Eichhorn)
    * `ESP8266WiFi.h` (Built-in ESP8266 core)

## Installation and Usage

1.  **Install Arduino IDE:** Download and install the latest Arduino IDE.
2.  **Install ESP8266 Board Package:** In Arduino IDE, go to `File > Preferences`, add `http://arduino.esp8266.com/stable/package_esp8266com_index.json` to "Additional Board Manager URLs". Then, go to `Tools > Board > Boards Manager...`, search for "esp8266", and install the package.
3.  **Install Libraries:**
    * In Arduino IDE, go to `Sketch > Include Library > Manage Libraries...`.
    * Search for and install:
        * "Adafruit Unified Sensor"
        * "Adafruit BME680 Library"
        * "SSD1306Wire"
4.  **Open the Sketch:** Copy the provided code into a new Arduino sketch.
5.  **Configure Wi-Fi AP:** You can modify `AP_SSID` and `AP_PASSWORD` at the top of the sketch to change your Wi-Fi hotspot name and password.
6.  **Select Board:** Go to `Tools > Board` and select your specific ESP8266 board (e.g., "NodeMCU 1.0 (ESP-12E Module)").
7.  **Select Flash Size:** Crucially, select an appropriate Flash Size (e.g., `4M (1M SPIFFS)` or `4M (FS:3MB, OTA:~512KB)`) to ensure enough space for the firmware. Memory errors can occur with smaller flash allocations.
8.  **Upload:** Connect your ESP8266 board and click the "Upload" button.
9.  **Connect to AP:** After uploading, open your smartphone or laptop's Wi-Fi settings. Connect to the Wi-Fi network named `ESP8266_BME680` (or your custom SSID) using the password `password123` (or your custom password).

## Gas Resistance Interpretation

The BME680 sensor provides raw gas resistance in kilo-Ohms (kOhm).

* **Higher kOhm values generally indicate better air quality.**
* **Lower kOhm values typically suggest higher concentrations of Volatile Organic Compounds (VOCs) or other pollutants.**

The `Gas Status` (Very Good, Good, Bad) displayed on the OLED is based on predefined thresholds in the code. To effectively interpret these values for your specific environment, it is highly recommended to:

1.  **Establish a Baseline:** Run the device continuously for at least 24-48 hours in your typical environment (e.g., your room with normal ventilation). Observe the range of kOhm values that correspond to what you consider "good" or "normal" air.
2.  **Adjust Thresholds (Optional):** If needed, you can modify the `getGasStatus` function in the code to set custom thresholds that are more relevant to your established baseline.

## Limitations & Compatibility Notes

* **ESP8266 Memory:** This code is optimized to fit on ESP8266. Due to severe memory constraints, advanced features like the BSEC IAQ algorithm or complex web interfaces with interactive settings forms are not included or enabled. Adding more complex features might lead to "out of memory" compilation errors.
* **ESP8266 Built-in OLED:** This code is specifically tailored for ESP8266 boards that have an SSD1306 OLED display already integrated and wired to the default I2C pins (GPIO 14 SDA, GPIO 12 SCL, I2C address 0x3C). Compatibility with other OLED wiring or displays may vary.

---
