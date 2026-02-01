/*
 * MC2 - Sensor Management
 * HC-SR04 Ultrasonic Sensors with Median Filter
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#pragma once

#include <Arduino.h>
#include "Config.h"

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct SensorData {
  int16_t distances[8];           // Filtered distances in mm
  bool sensorActive[8];           // Which sensors are active
  bool dataReady;                 // New data available
  unsigned long lastUpdate;       // Timestamp of last measurement
};

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Setup
void setupSensors();

// Sensor Activation
void setSensorActive(uint8_t sensor, bool active);
void setMultipleSensorsActive(const uint8_t* sensors, uint8_t count, bool active);

// Sensor Task (runs on Core 1)
void sensorTask(void* parameter);

// Single Measurement
uint16_t measureDistance(uint8_t sensor);

// Filtering
int16_t calculateFilteredValue(int16_t samples[], uint8_t numSamples);
void sortArray(int16_t arr[], uint8_t size);

// Data Retrieval (Thread-safe)
void getSensorData(int16_t* distArray);
bool isSensorDataReady();

// Calibration
void applyCalibration(int16_t* distances);
