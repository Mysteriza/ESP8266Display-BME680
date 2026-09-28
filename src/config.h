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
#define BME_HEALING_MS 5000

// Timing Configuration
#define SENSOR_READ_INTERVAL_MS 30000UL
#define OLED_DATA_SCREEN_1_DURATION 5000UL
#define OLED_DATA_SCREEN_2_DURATION 5000UL
#define OLED_DATA_SCREEN_3_DURATION 5000UL
#define OLED_DATA_SCREEN_4_DURATION 5000UL

// Power Management Thresholds
#define BATTERY_CRITICAL_MV 3200
#define BATTERY_LOW_MV 3500
#define BATTERY_NORMAL_MV 3700

// Thermal Protection
#define HOT_ENTER_C 45.0f
#define HOT_EXIT_C 41.0f
#define SAFETY_PERIOD_MS 60000UL

// BSEC Configuration
#define BSEC_SAVE_INTERVAL_MS 14400000UL
#define BSEC_MIN_SAVE_GAP_MS 600000UL
#define BSEC_BOOT_STABILIZATION_MS 300000UL

// IAQ Filtering
#define IAQ_VAR_ALPHA 0.10f
#define IAQ_DISPLAY_MIN_ACCURACY 2

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
#define EEPROM_SIZE 512
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

#endif
