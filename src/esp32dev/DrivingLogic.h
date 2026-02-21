/*
 * MC2 - Driving Logic 
 * State Machine for autonomous navigation with dynamic sensor usage
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-28
 * Version: 2.0
 */

#pragma once

#include <Arduino.h>
#include "Config.h"
#include "UserSettings.h"
#include "I2C_Communication.h"
#include "Display.h"
#include "Sensors.h"

// ============================================================================
// ENUMERATIONS
// ============================================================================

// Driving States
enum DriveState {
  STATE_INIT,           // Initialization: Read all sensors, make decision
  STATE_READY,          // Ready to drive
  STATE_DRIVING,        // Actively driving
  STATE_OBSTACLE,       // Obstacle detected, stopped
  STATE_ERROR           // Error (e.g. no wall detected)
};

// Wall Side
enum WallSide { 
  NO_WALL, 
  LEFT_WALL, 
  RIGHT_WALL 
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Structure for wall distance measurement
struct WallMeasurement {
  int16_t frontDistance;   // Distance front sensor
  int16_t rearDistance;    // Distance rear sensor
  float angleToWall;       // Angle to wall (in degrees)
  int16_t avgDistance;     // Average distance
  bool valid;              // Measurement valid
  
  // New fields for intelligent wall detection
  bool frontJumped;        // Front sensor jumped 50%+ or to error
  bool rearJumped;         // Rear sensor jumped 50%+ or to error
  bool bothLost;           // Both sensors lost wall
};

// Structure for system state
struct SystemState {
  DriveState state;        // Current state
  WallSide selectedWall;   // Selected wall
  bool initialized;        // System initialized
  int16_t currentSpeed;    // Current speed
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

extern SystemState systemState;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Main Function
void autonomousDrive(const int16_t* distances);

// Initialization
void initializeDriving(const int16_t* distances);
WallSide detectWall(const int16_t* distances);
void setupTestMode();

// Sensor Management
void activateSensorsForWall(WallSide wall);
void activateAllSensors();

// Driving Logic
void executeDriving(const int16_t* distances);
int16_t calculateDynamicSpeed(const int16_t* distances);
WallMeasurement measureWall(const int16_t* distances, WallSide side);
int16_t calculateSteering(const WallMeasurement& wall);
void resetSensorHistory();
void resetSteeringIntegral();  // Resets I-component

// Helper Functions
int16_t calculateMedian(int16_t* array, int size);

// Test Sequence (for development)
void runTestSequence();
