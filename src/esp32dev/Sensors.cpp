/*
 * MC2 - Sensor Implementation
 * HC-SR04 Ultrasonic Sensors with Median Filter
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#include "Sensors.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Definition of externally declared arrays from Config.h
const char* SENSOR_NAMES[8] = {
  "L-Side-R",
  "L-Rear  ",
  "R-Rear  ",
  "R-Side-R",
  "R-Side-F",
  "R-Front ",
  "L-Side-F",
  "L-Front "
};

const int16_t SENSOR_OFFSETS[8] = {
  -20,  // L-Side-Rear
  -20,  // L-Rear
  -20,  // R-Rear
  -20,  // R-Side-Rear
  -20,  // R-Side-Front
  -20,  // R-Front
  -20,  // L-Side-Front
  -20   // L-Front
};

SensorData sensorData;
SemaphoreHandle_t sensorMutex;
int16_t rawSamples[8][NUM_SAMPLES];  // Raw data: [Sensor][Sample]

// ============================================================================
// SETUP
// ============================================================================

void setupSensors() {
  // Create mutex
  sensorMutex = xSemaphoreCreateMutex();
  
  // Initialize sensor pins
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    pinMode(ECHO_PINS[i], INPUT);
    sensorData.sensorActive[i] = true;
    sensorData.distances[i] = 0;
  }
  
  sensorData.dataReady = false;
  sensorData.lastUpdate = 0;
}

// ============================================================================
// SENSOR ACTIVATION
// ============================================================================

void setSensorActive(uint8_t sensor, bool active) {
  if (sensor >= 8) return;
  
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    sensorData.sensorActive[sensor] = active;
    xSemaphoreGive(sensorMutex);
  }
}

void setMultipleSensorsActive(const uint8_t* sensors, uint8_t count, bool active) {
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    for (uint8_t i = 0; i < count; i++) {
      if (sensors[i] < 8) {
        sensorData.sensorActive[sensors[i]] = active;
      }
    }
    xSemaphoreGive(sensorMutex);
  }
}

// ============================================================================
// SINGLE MEASUREMENT
// ============================================================================

uint16_t measureDistance(uint8_t sensor) {
  if (!sensorData.sensorActive[sensor] || sensor >= 8) {
    return 0;
  }
  
  digitalWrite(TRIG_PINS[sensor], LOW);
  delayMicroseconds(TRIG_PULSE_PRE_US);
  digitalWrite(TRIG_PINS[sensor], HIGH);
  delayMicroseconds(TRIG_PULSE_WIDTH_US);
  digitalWrite(TRIG_PINS[sensor], LOW);
  
  unsigned long duration = pulseIn(ECHO_PINS[sensor], HIGH, TIMEOUT_US);
  
  if (duration == 0) {
    return 0;
  }
  
  uint16_t distance_mm = (duration * 343) / 2000;
  
  if (distance_mm > MAX_DISTANCE_MM) {
    return 0;
  }
  
  return distance_mm;
}

// ============================================================================
// FILTERING
// ============================================================================

void sortArray(int16_t arr[], uint8_t size) {
  for (uint8_t i = 0; i < size - 1; i++) {
    for (uint8_t j = 0; j < size - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int16_t temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

int16_t calculateFilteredValue(int16_t samples[], uint8_t numSamples) {
  // Check for errors/timeouts
  uint8_t validCount = 0;
  for (uint8_t i = 0; i < numSamples; i++) {
    if (samples[i] > 0) validCount++;
  }
  
  if (validCount < 3) {
    return -1;  // Too many errors
  }
  
  // Sort array
  int16_t sorted[NUM_SAMPLES];
  memcpy(sorted, samples, sizeof(sorted));
  sortArray(sorted, numSamples);
  
  // Find first and last valid values (> 0)
  int8_t firstValid = -1;
  int8_t lastValid = -1;
  
  for (uint8_t i = 0; i < numSamples; i++) {
    if (sorted[i] > 0) {
      if (firstValid == -1) firstValid = i;
      lastValid = i;
    }
  }
  
  // Discard min (firstValid) and max (lastValid)
  // Average the values in between
  int32_t sum = 0;
  uint8_t count = 0;
  
  for (int8_t i = firstValid + 1; i < lastValid; i++) {
    if (sorted[i] > 0) {
      sum += sorted[i];
      count++;
    }
  }
  
  if (count == 0) {
    return -1;
  }
  
  return (int16_t)(sum / count);
}

// ============================================================================
// CALIBRATION
// ============================================================================

void applyCalibration(int16_t* distances) {
  for (int i = 0; i < 8; i++) {
    if (distances[i] > 0) {
      distances[i] = distances[i] + SENSOR_OFFSETS[i];
    }
  }
}

// ============================================================================
// SENSOR TASK (runs on Core 1)
// ============================================================================

void sensorTask(void* parameter) {
  while (true) {
    // List of active sensors (thread-safe)
    uint8_t activeSensors[8];
    uint8_t activeCount = 0;
    
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      for (uint8_t i = 0; i < 8; i++) {
        if (sensorData.sensorActive[i]) {
          activeSensors[activeCount++] = i;
        }
      }
      xSemaphoreGive(sensorMutex);
    }
    
    // 5 passes - only through active sensors
    for (uint8_t pass = 0; pass < NUM_SAMPLES; pass++) {
      // Measure only active sensors
      for (uint8_t i = 0; i < activeCount; i++) {
        uint8_t sensor = activeSensors[i];
        uint16_t dist = measureDistance(sensor);
        rawSamples[sensor][pass] = (dist == 0) ? -1 : dist;
        
        // Short pause between sensors (prevent echo crosstalk)
        if (i < activeCount - 1) {  // Not after last sensor
          delay(2);
        }
      }
      
      // Mark inactive sensors
      for (uint8_t sensor = 0; sensor < 8; sensor++) {
        bool isActive = false;
        for (uint8_t j = 0; j < activeCount; j++) {
          if (activeSensors[j] == sensor) {
            isActive = true;
            break;
          }
        }
        if (!isActive) {
          rawSamples[sensor][pass] = -2;
        }
      }
      
      delay(MEASUREMENT_INTERVAL);  // Pause after each pass (60ms)
    }
    
    // Calculate filtered values
    int16_t tempDistances[8];
    for (uint8_t sensor = 0; sensor < 8; sensor++) {
      if (sensorData.sensorActive[sensor]) {
        tempDistances[sensor] = calculateFilteredValue(rawSamples[sensor], NUM_SAMPLES);
      } else {
        tempDistances[sensor] = -2;
      }
    }
    
    // Apply calibration
    applyCalibration(tempDistances);
    
    // Write to global structure (thread-safe)
    if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(sensorData.distances, tempDistances, sizeof(tempDistances));
      sensorData.dataReady = true;
      sensorData.lastUpdate = millis();
      xSemaphoreGive(sensorMutex);
    }
    
    // Short pause before next cycle
    delay(50);
  }
}

// ============================================================================
// THREAD-SAFE DATA ACCESS
// ============================================================================

void getSensorData(int16_t* distArray) {
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(distArray, sensorData.distances, sizeof(sensorData.distances));
    sensorData.dataReady = false;  // Reset flag
    xSemaphoreGive(sensorMutex);
  }
}

bool isSensorDataReady() {
  bool ready = false;
  if (xSemaphoreTake(sensorMutex, 10) == pdTRUE) {
    ready = sensorData.dataReady;
    xSemaphoreGive(sensorMutex);
  }
  return ready;
}
