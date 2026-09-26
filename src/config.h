#ifndef CONFIG_H
#define CONFIG_H

#include <bsec.h>

// I2C Configuration
#define I2C_SDA_GPIO 14
#define I2C_SCL_GPIO 12
#define I2C_CLOCK_SLOW 100000UL
#define I2C_CLOCK_FAST 400000UL

// Display Configuration
#define OLED_ADDRESS 0x3C
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_RESET -1

// Sensor Configuration
#define BME_ADDRESS 0x76

// Timing Configuration
#define SENSOR_READ_INTERVAL_MS 30000UL
#define OLED_DATA_SCREEN_1_DURATION 5000UL
#define OLED_DATA_SCREEN_2_DURATION 5000UL
#define OLED_DATA_SCREEN_3_DURATION 5000UL
#define OLED_DATA_SCREEN_4_DURATION 5000UL

// Thermal Protection
#define HOT_ENTER_C 45.0f
#define HOT_EXIT_C 41.0f
#define SAFETY_PERIOD_MS 60000UL

// BSEC Configuration
#define BSEC_SAVE_INTERVAL_MS 14400000UL
#define BSEC_MIN_SAVE_GAP_MS 600000UL

// IAQ Filtering
#define IAQ_VAR_ALPHA 0.10f

// Gas Resistance Baseline
#define GAS_EMA_ALPHA 0.20f
#define BASELINE_ALPHA_UP 0.02f
#define BASELINE_ALPHA_DOWN 0.001f
#define GAS_BASELINE_READY_SAMPLES 2880

// Transport Detection
#define PORT_DECIM_N 10
#define PORT_BUF 24
#define PORT_TH_DP 3.5f
#define PORT_TH_DH 8.0f
#define PORT_TH_DIAQ 25.0f
#define PORT_RESUME_OK 6

// Altitude Filtering
#define ALT_EMA_ALPHA 0.12f
#define ALT_OUTLIER_M 18.0f
#define ALT_DEADBAND_M 0.40f

// Display Update Thresholds
#define TH_T 0.10f
#define TH_H 0.10f
#define TH_P 0.10f
#define TH_ALT 1.0f
#define TH_G 0.10f
#define TH_IAQ 0.50f

// OLED Contrast
#define CONTRAST_NORMAL 0x7F
#define CONTRAST_OVERHEAT 0x04

// EEPROM Memory Map
#define EEPROM_SIZE 1024
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_MAGIC_VALUE 0xB6680A11
#define SEA_LEVEL_PRESSURE_ADDR 4
#define GAS_BASELINE_ADDR 8
#define GAS_BASELINE_READY_ADDR 12
#define BSEC_STATE_ADDR 16
#define BSEC_STATE_MAXLEN BSEC_MAX_STATE_BLOB_SIZE
#define BSEC_STATE_VALID_ADDR (BSEC_STATE_ADDR + BSEC_STATE_MAXLEN)
#define BSEC_STATE_VALID_MAGIC 0xB5EC1A0F

// Boot and timeout constants
#define BOOT_GRACE_MS 60000UL
#define NO_DATA_TIMEOUT_BOOT_MS 45000UL
#define NO_DATA_TIMEOUT_RUN_MS 20000UL

// WiFi Periodic Sync (Option B: radio on only for sync window)
#define WIFI_SSID_LEN 33 // 32 chars + null
#define WIFI_PASS_LEN 65 // 64 chars + null
#define WIFI_MAX_NETS 3  // Stored networks, tried in slot priority order
#define WIFI_HOSTNAME "BME680-Monitor"
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#define WIFI_MAX_ATTEMPTS 3      // Burst retries per sync window (~1 min total)
#define WIFI_RETRY_GAP_MS 2000UL // Radio rest between attempts in a burst
#define WIFI_FETCH_TIMEOUT_MS 10000UL
#define WIFI_SYNC_INTERVAL_MS 3600000UL  // Hourly QNH refresh
#define WIFI_FIRST_SYNC_DELAY_MS 30000UL // First attempt 30s after boot
#define WIFI_RETRY_FAIL_MS 900000UL      // Retry 15min after failure
#define WIFI_BODY_MAX 1024               // Open-Meteo current= response buffer
#define WIFI_API_HOST "api.open-meteo.com"

// Default coordinates (overridable via serial LAT/LON, persisted)
#define DEFAULT_LAT -6.8982836f
#define DEFAULT_LON 107.6349833f

// Auto-QNH policy
#define QNH_AUTO_DEADBAND_HPA 0.4f // Ignore API jitter below this (~3m)

// QNH valid range, single source of truth (sea-level pressure, hPa)
#define QNH_MIN_HPA 870.0f
#define QNH_MAX_HPA 1100.0f

// WiFi EEPROM block (starts clear of BSEC blob which ends at ~175)
#define WIFI_MAGIC_ADDR 192
#define WIFI_MAGIC_VALUE 0x57494649
#define WIFI_SSID_ADDR 196
#define WIFI_PASS_ADDR 229
#define WIFI_LAT_ADDR 294
#define WIFI_LON_ADDR 298
#define WIFI_FLAGS_ADDR 302
#define WIFI_FLAG_AUTO 0x01
#define WIFI_FLAG_AUTOSRC 0x02
// Extra network slots (slot 0 = legacy addresses above)
#define WIFI2_SSID_ADDR 320
#define WIFI2_PASS_ADDR 353
#define WIFI3_SSID_ADDR 424
#define WIFI3_PASS_ADDR 457

#endif
