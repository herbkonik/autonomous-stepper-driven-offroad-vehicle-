/*
 * MC2 - Main Program (Dual-Core) 
 * 
 * Hardware:
 * - ESP32-WROOM-32U (Dual-Core!)
 * - 8× HC-SR04 (CS100A) Ultrasonic Sensors
 * - 2.0" TFT Display ST7789VW (240×320)
 * 
 * Core 0 (Main):
 * - Display Updates
 * - I2C Communication with MC1
 * - Driving Logic
 * 
 * Core 1 (Sensors):
 * - HC-SR04 Measurements
 * - Median Filter
 * - Calibration
 * 
 * Communication:
 * - Mutex-protected sensor data
 * - Thread-safe access
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24 
 * Version: 1.0 (Dual-Core Architecture)
 */

#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "Display.h"
#include "I2C_Communication.h"
#include "DrivingLogic.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

TaskHandle_t sensorTaskHandle;
int16_t currentDistances[8];
unsigned long lastDisplayUpdate = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialization on Core 0
  
  // Initialize display
  showInitStatus("Init Display...");
  setupDisplay();
  delay(1000);
  
  // Initialize I2C
  showInitStatus("Init I2C...");
  setupI2C();
  delay(500);
  
  // Initialize sensors
  showInitStatus("Init Sensors...");
  setupSensors();
  delay(500);
  
  // Initialize test mode
  showInitStatus("Init Test Mode...");
  setupTestMode();
  delay(500);
  
  // Start sensor task on Core 1
  showInitStatus("Start Core 1...");
  xTaskCreatePinnedToCore(
    sensorTask,              // Task function
    "SensorTask",            // Name
    SENSOR_TASK_STACK,       // Stack size
    NULL,                    // Parameter
    SENSOR_TASK_PRIORITY,    // Priority
    &sensorTaskHandle,       // Task handle
    CORE_SENSORS             // Core 1
  );
  
  delay(1000);
  
  // Ready - normal operation
  setupDisplay();
  showDriveStatus("System Ready");
}

// ============================================================================
// MAIN LOOP (runs on Core 0)
// ============================================================================

void loop() {
  // Display update (only when new data available)
  if (isSensorDataReady()) {
    getSensorData(currentDistances);
    
    // Update display (rate-limited)
    if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
      updateSensorDisplay(currentDistances);
      lastDisplayUpdate = millis();
    }
  }
  
  // Test sequence
  // runTestSequence();
  
  // Autonomous navigation
   autonomousDrive(currentDistances);
  
  // Short pause for other tasks
  delay(10);
}

// ============================================================================
// END
// ============================================================================
