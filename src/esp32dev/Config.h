/*
 * MC2 - Configuration File
 * All constants, pins and settings
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#pragma once

#include <Arduino.h>

// ============================================================================
// HC-SR04 SENSOR CONFIGURATION
// ============================================================================

// Trigger Pins
constexpr uint8_t TRIG_PINS[8] = {
  32,  // Sensor 0: L-Side-Rear 
  25,  // Sensor 1: L-Rear 
  27,  // Sensor 2: R-Rear 
  12,  // Sensor 3: R-Side-Rear 
  15,  // Sensor 4: R-Side-Front 
  4,   // Sensor 5: R-Front 
  13,  // Sensor 6: L-Side-Front 
  5    // Sensor 7: L-Front 
};

// Echo Pins (uses INPUT_ONLY pins: 35, 34, 39/VN, 36/VP)
constexpr uint8_t ECHO_PINS[8] = {
  35,  // Sensor 0: L-Side-Rear (INPUT_ONLY)
  33,  // Sensor 1: L-Rear 
  26,  // Sensor 2: R-Rear 
  14,  // Sensor 3: R-Side-Rear 
  2,   // Sensor 4: R-Side-Front 
  39,  // Sensor 5: R-Front (VN - INPUT_ONLY!)
  36,  // Sensor 6: L-Side-Front (VP - INPUT_ONLY!)
  34   // Sensor 7: L-Front (INPUT_ONLY)
};

// Sensor Names (external declaration - definition in Sensors.cpp)
extern const char* SENSOR_NAMES[8];

// Sensor Offsets (calibration in mm) (external declaration - definition in Sensors.cpp)
extern const int16_t SENSOR_OFFSETS[8];

// Ultrasonic Parameters
constexpr uint16_t MAX_DISTANCE_MM = 4000;
constexpr uint32_t TIMEOUT_US = 23200;
constexpr uint16_t MEASUREMENT_INTERVAL = 60;
constexpr uint8_t NUM_SAMPLES = 5;              // Number of measurements per sensor

// HC-SR04 Trigger Pulse Configuration
constexpr uint8_t TRIG_PULSE_PRE_US = 2;
constexpr uint8_t TRIG_PULSE_WIDTH_US = 12;

// ============================================================================
// I2C CONFIGURATION
// ============================================================================

constexpr uint8_t MC1_I2C_ADDRESS = 0x10;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint32_t I2C_CLOCK = 100000;  // 100kHz

// MC1 Register Addresses
constexpr uint8_t REG_SPEED = 0x01;
constexpr uint8_t REG_STEERING = 0x02;
constexpr uint8_t REG_COMMAND = 0x03;

// MC1 Command Codes
constexpr uint8_t CMD_STOP = 0x01;
constexpr uint8_t CMD_EMERGENCY_STOP = 0xFF;
constexpr uint8_t CMD_CENTER_STEERING = 0x02;

// ============================================================================
// DISPLAY CONFIGURATION
// ============================================================================

// TFT Colors (RGB565)
constexpr uint16_t COLOR_BG       = 0x0000;  // Black
constexpr uint16_t COLOR_TEXT     = 0xFFFF;  // White
constexpr uint16_t COLOR_HEADER   = 0x07E0;  // Green
constexpr uint16_t COLOR_OK       = 0x07E0;  // Green
constexpr uint16_t COLOR_WARNING  = 0xFFE0;  // Yellow
constexpr uint16_t COLOR_ERROR    = 0xF800;  // Red
constexpr uint16_t COLOR_INACTIVE = 0x7BEF;  // Gray
constexpr uint16_t COLOR_US       = 0x07FF;  // Cyan

// ============================================================================
// DUAL-CORE CONFIGURATION
// ============================================================================

constexpr uint8_t CORE_MAIN = 0;      // Display, I2C, Driving Logic
constexpr uint8_t CORE_SENSORS = 1;   // Sensor Measurements

constexpr uint16_t SENSOR_TASK_STACK = 4096;
constexpr uint8_t SENSOR_TASK_PRIORITY = 1;

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

constexpr uint16_t DISPLAY_UPDATE_INTERVAL = 200;  // ms
constexpr uint16_t TEST_SEQUENCE_INTERVAL = 3000;  // ms
