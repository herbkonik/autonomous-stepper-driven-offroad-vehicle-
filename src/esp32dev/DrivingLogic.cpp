/*
 * MC2 - Driving Logic Implementation
 * State Machine for autonomous navigation with dynamic sensor usage
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-02-10
 * Version: 2.1 (PI Controller for angle control)
 */

#include "DrivingLogic.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

SystemState systemState = {
  .state = STATE_INIT,
  .selectedWall = NO_WALL,
  .initialized = false,
  .currentSpeed = 0
};

// PI Controller Variables
static float angleErrorIntegral = 0.0;
static unsigned long lastSteeringUpdateTime = 0;

// ============================================================================
// TEST SEQUENCE (unchanged from previous version)
// ============================================================================

void runTestSequence() {
  static unsigned long lastTest = 0;
  static byte testPhase = 0;
  
  if (millis() - lastTest >= TEST_SEQUENCE_INTERVAL) {
    lastTest = millis();
    testPhase++;
    
    switch (testPhase) {
    case 1:
      // Drive forward
      showDriveStatus("Forward 200");
      driveForward(200);
      setSteering(0);
      break;
      
    case 2:
      // Steer right
      showDriveStatus("Steer right 20");
      setSteering(20);
      break;
      
    case 3:
      // Straight
      showDriveStatus("Straight");
      setSteering(0);
      break;
      
    case 4:
      // Steer left
      showDriveStatus("Steer left 20");
      setSteering(-20);
      break;
      
    case 5:
      // Stop
      showDriveStatus("Stop");
      stopVehicle();
      centerSteering();
      testPhase = 0;
      break;
    }
  }
}

// ============================================================================
// SENSOR MANAGEMENT
// ============================================================================

void activateAllSensors() {
  for (uint8_t i = 0; i < 8; i++) {
    setSensorActive(i, true);
  }
}

void activateSensorsForWall(WallSide wall) {
  // First deactivate all
  for (uint8_t i = 0; i < 8; i++) {
    setSensorActive(i, false);
  }
  
  // Sensor indices (from Config.h):
  // 0: L-Side-R, 1: L-Rear, 2: R-Rear, 3: R-Side-R
  // 4: R-Side-F, 5: R-Front, 6: L-Side-F, 7: L-Front
  
  if (wall == LEFT_WALL) {
    // Left wall: L-Side-F (6), L-Side-R (0), L-Front (7), R-Front (5)
    setSensorActive(0, true);  // L-Side-R
    setSensorActive(6, true);  // L-Side-F
    setSensorActive(7, true);  // L-Front
    setSensorActive(5, true);  // R-Front
    
  } else if (wall == RIGHT_WALL) {
    // Right wall: R-Side-F (4), R-Side-R (3), L-Front (7), R-Front (5)
    setSensorActive(3, true);  // R-Side-R
    setSensorActive(4, true);  // R-Side-F
    setSensorActive(7, true);  // L-Front
    setSensorActive(5, true);  // R-Front
  }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

WallSide detectWall(const int16_t* distances) {
  // Sensor indices (from Config.h):
  // 0: L-Side-R, 1: L-Rear, 2: R-Rear, 3: R-Side-R
  // 4: R-Side-F, 5: R-Front, 6: L-Side-F, 7: L-Front
  
  // Take 5 measurements for side sensors with 20ms pause for robust detection
  const int NUM_MEASUREMENTS = 5;
  const int MEASUREMENT_DELAY = 20; // ms between measurements
  
  int16_t leftSideFront[NUM_MEASUREMENTS];
  int16_t leftSideRear[NUM_MEASUREMENTS];
  int16_t rightSideFront[NUM_MEASUREMENTS];
  int16_t rightSideRear[NUM_MEASUREMENTS];
  
  // Perform 5 measurement rounds with fresh sensor data each time
  for (int round = 0; round < NUM_MEASUREMENTS; round++) {
    // Get fresh sensor data from Core 0
    int16_t freshDistances[8];
    getSensorData(freshDistances);
    
    // Store side sensor values
    leftSideFront[round] = freshDistances[6];  // L-Side-F
    leftSideRear[round] = freshDistances[0];   // L-Side-R
    rightSideFront[round] = freshDistances[4]; // R-Side-F
    rightSideRear[round] = freshDistances[3];  // R-Side-R
    
    if (round < NUM_MEASUREMENTS - 1) {
      delay(MEASUREMENT_DELAY);  // Give Core 0 time for next measurement
    }
  }
  
  // Calculate median for each sensor
  int16_t leftSideFrontMedian = calculateMedian(leftSideFront, NUM_MEASUREMENTS);
  int16_t leftSideRearMedian = calculateMedian(leftSideRear, NUM_MEASUREMENTS);
  int16_t rightSideFrontMedian = calculateMedian(rightSideFront, NUM_MEASUREMENTS);
  int16_t rightSideRearMedian = calculateMedian(rightSideRear, NUM_MEASUREMENTS);
  
  // Check if enough valid measurements (3 out of 5 must be > 0)
  int leftValidCount = 0;
  if (leftSideFrontMedian > 0) leftValidCount++;
  if (leftSideRearMedian > 0) leftValidCount++;
  
  int rightValidCount = 0;
  if (rightSideFrontMedian > 0) rightValidCount++;
  if (rightSideRearMedian > 0) rightValidCount++;
  
  if (leftValidCount < 2) {  // Need at least 2 valid sensors per side
    return NO_WALL;  // Not enough valid left measurements
  }
  if (rightValidCount < 2) {
    return NO_WALL;  // Not enough valid right measurements
  }
  
  // Calculate average for each side from valid measurements
  int16_t leftSide = 0, rightSide = 0;
  int leftCount = 0, rightCount = 0;
  
  if (leftSideFrontMedian > 0) {
    leftSide += leftSideFrontMedian;
    leftCount++;
  }
  if (leftSideRearMedian > 0) {
    leftSide += leftSideRearMedian;
    leftCount++;
  }
  if (leftCount > 0) leftSide /= leftCount;
  
  if (rightSideFrontMedian > 0) {
    rightSide += rightSideFrontMedian;
    rightCount++;
  }
  if (rightSideRearMedian > 0) {
    rightSide += rightSideRearMedian;
    rightCount++;
  }
  if (rightCount > 0) rightSide /= rightCount;
  
  // Validation (ignore errors, only valid ranges)
  bool leftValid = (leftSide > WALL_DETECTION_MIN && leftSide < WALL_DETECTION_MAX);
  bool rightValid = (rightSide > WALL_DETECTION_MIN && rightSide < WALL_DETECTION_MAX);
  
  if (!leftValid && !rightValid) {
    return NO_WALL;  // No wall detected
  }
  
  if (leftValid && !rightValid) {
    return LEFT_WALL;  // Only left wall
  }
  
  if (rightValid && !leftValid) {
    return RIGHT_WALL;  // Only right wall
  }
  
  // Both walls detected -> choose CLOSER wall (more stable)
  if (leftSide < rightSide) {
    return LEFT_WALL;   // Left wall is closer
  } else {
    return RIGHT_WALL;  // Right wall is closer or equal
  }
}

// Helper function to calculate median of an array
int16_t calculateMedian(int16_t* array, int size) {
  // Simple bubble sort for small arrays
  for (int i = 0; i < size - 1; i++) {
    for (int j = 0; j < size - i - 1; j++) {
      if (array[j] > array[j + 1]) {
        int16_t temp = array[j];
        array[j] = array[j + 1];
        array[j + 1] = temp;
      }
    }
  }
  
  // Return median (middle element for odd size)
  return array[size / 2];
}

void initializeDriving(const int16_t* distances) {
  // Detect wall
  systemState.selectedWall = detectWall(distances);
  
  if (systemState.selectedWall == NO_WALL) {
    systemState.state = STATE_ERROR;
    showDriveStatus("ERROR: No wall!");
    stopVehicle();
    centerSteering();
    return;
  }
  
  // Activate sensors for selected wall
  activateSensorsForWall(systemState.selectedWall);
  
  // Reset PI controller for restart
  resetSteeringIntegral();
  
  // Show status
  if (systemState.selectedWall == LEFT_WALL) {
    showDriveStatus("Init: Follow L-Wall");
  } else {
    showDriveStatus("Init: Follow R-Wall");
  }
  
  // Change state
  systemState.state = STATE_READY;
  systemState.initialized = true;
  
  delay(500);  // Short pause for sensor adjustment
  
  systemState.state = STATE_DRIVING;
}

// ============================================================================
// DYNAMIC SPEED CONTROL
// ============================================================================

int16_t calculateDynamicSpeed(const int16_t* distances) {
  // Get fresh sensor data from Core 0
  int16_t freshDistances[8];
  getSensorData(freshDistances);
  
  // Front sensors
  int16_t frontLeft = freshDistances[7];   // L-Front
  int16_t frontRight = freshDistances[5];  // R-Front
  
  // For front sensors: No echo = no obstacle = maximum distance
  // Treat invalid/timeout values as maximum distance (free path)
  if (frontLeft <= 0) frontLeft = MAX_DISTANCE_MM;
  if (frontRight <= 0) frontRight = MAX_DISTANCE_MM;
  
  // Find smallest valid distance (closest obstacle)
  int16_t minFrontDistance = min(frontLeft, frontRight);
  
  // Speed based on distance
  int16_t targetSpeed = driveSpeed;
  
  if (minFrontDistance < SPEED_STOP) {
    targetSpeed = 0;  // STOP
  } else if (minFrontDistance < SPEED_REDUCE_75) {
    targetSpeed = driveSpeed * 0.25;  // 25% speed
  } else if (minFrontDistance < SPEED_REDUCE_50) {
    targetSpeed = driveSpeed * 0.50;  // 50% speed
  } else if (minFrontDistance < SPEED_REDUCE_25) {
    targetSpeed = driveSpeed * 0.75;  // 75% speed
  } else if (minFrontDistance < SPEED_REDUCE_START) {
    targetSpeed = driveSpeed * 0.90;  // 90% speed
  }
  // else: full speed (no obstacle detected)
  
  return targetSpeed;
}

// ============================================================================
// WALL-FOLLOWING LOGIC
// ============================================================================

WallMeasurement measureWall(const int16_t* distances, WallSide side) {
  WallMeasurement wall;
  wall.valid = false;
  
  // Get fresh sensor data from Core 0
  int16_t freshDistances[8];
  getSensorData(freshDistances);
  
  if (side == LEFT_WALL) {
    // Left wall: Front = 6 (L-Side-F), Rear = 0 (L-Side-R)
    wall.frontDistance = freshDistances[6];
    wall.rearDistance = freshDistances[0];
    
  } else if (side == RIGHT_WALL) {
    // Right wall: Front = 4 (R-Side-F), Rear = 3 (R-Side-R)
    wall.frontDistance = freshDistances[4];
    wall.rearDistance = freshDistances[3];
    
  } else {
    return wall;  // NO_WALL
  }
  
  // Validation
  bool frontValid = (wall.frontDistance > 0);
  bool rearValid = (wall.rearDistance > 0);
  
  // Both sensors invalid → error
  if (!frontValid && !rearValid) {
    return wall;  // Both sensors error
  }
  
  // CORNERING: Only front sensor has echo (rear loses wall)
  if (frontValid && !rearValid) {
    wall.avgDistance = wall.frontDistance;
    wall.angleToWall = 0.0;  // No angle calculable, only distance
    wall.valid = true;
    return wall;
  }
  
  // Only rear sensor (shouldn't happen, but for safety)
  if (!frontValid && rearValid) {
    wall.avgDistance = wall.rearDistance;
    wall.angleToWall = 0.0;
    wall.valid = true;
    return wall;
  }
  
  // NORMAL CASE: Both sensors have echo
  // Average distance
  wall.avgDistance = (wall.frontDistance + wall.rearDistance) / 2;
  
  // Calculate angle to wall
  int16_t deltaDistance = wall.frontDistance - wall.rearDistance;
  wall.angleToWall = atan2(deltaDistance, SIDE_SENSOR_DISTANCE) * 57.2958;  // Rad -> Deg
  
  wall.valid = true;
  return wall;
}

int16_t calculateSteering(const WallMeasurement& wall) {
  if (!wall.valid) {
    return 0;
  }
  
  // Time difference calculation (for I-component)
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastSteeringUpdateTime) / 1000.0;  // in seconds
  
  // First call: initialize time
  if (lastSteeringUpdateTime == 0) {
    lastSteeringUpdateTime = currentTime;
    deltaTime = 0.0;
  } else {
    lastSteeringUpdateTime = currentTime;
  }
  
  // 1. DISTANCE CONTROL (P Controller)
  int16_t distanceError = wall.avgDistance - targetWallDistance;
  
  // Only correct if outside tolerance
  int16_t distanceSteering = 0;
  if (abs(distanceError) > DISTANCE_TOLERANCE) {
    distanceSteering = -(int16_t)(distanceError * DISTANCE_P_GAIN);
  }
  
  // 2. ANGLE CONTROL (PI Controller)
  float angleSteering = 0.0;
  
  // Only correct if outside dead zone
  if (abs(wall.angleToWall) > MIN_ANGLE_CORRECTION) {
    // P-component: Proportional to current angle error
    float angleP = -wall.angleToWall * ANGLE_P_GAIN;
    
    // I-component: Integral of angle error over time
    if (deltaTime > 0) {
      angleErrorIntegral += (-wall.angleToWall) * deltaTime;
      
      // Anti-Windup: Limit integral
      angleErrorIntegral = constrain(angleErrorIntegral, -ANGLE_I_LIMIT / ANGLE_I_GAIN, 
                                                          ANGLE_I_LIMIT / ANGLE_I_GAIN);
    }
    
    float angleI = angleErrorIntegral * ANGLE_I_GAIN;
    
    // Combined angle component (PI)
    angleSteering = angleP + angleI;
    
  } else {
    // Inside dead zone: Reset integral (Anti-Windup)
    angleErrorIntegral *= 0.95;  // Slowly decay
  }
  
  // 3. COMBINATION with weighting
  float combinedSteering = (1.0 - ANGLE_WEIGHT) * distanceSteering + 
                           ANGLE_WEIGHT * angleSteering;
  
  // Limiting
  int16_t finalSteering = (int16_t)combinedSteering;
  finalSteering = constrain(finalSteering, -maxSteeringAngle, maxSteeringAngle);
  
  return finalSteering;
}

// ============================================================================
// RESET INTEGRAL
// ============================================================================

void resetSteeringIntegral() {
  angleErrorIntegral = 0.0;
  lastSteeringUpdateTime = 0;
}

// ============================================================================
// EXECUTE DRIVING
// ============================================================================

void executeDriving(const int16_t* distances) {
  // Calculate dynamic speed
  int16_t targetSpeed = calculateDynamicSpeed(distances);
  
  // If speed 0 -> Stopped due to obstacle
  if (targetSpeed == 0) {
    systemState.state = STATE_OBSTACLE;
    showDriveStatus("STOP: Obstacle!");
    stopVehicle();
    centerSteering();
    return;
  }
  
  // Perform wall measurement
  WallMeasurement wall = measureWall(distances, systemState.selectedWall);
  
  if (!wall.valid) {
    showDriveStatus("Sensor Error");
    stopVehicle();
    return;
  }
  
  // Calculate steering angle (combines distance + angle)
  int16_t steeringAngle = calculateSteering(wall);
  
  // For right wall: Invert sign
  if (systemState.selectedWall == RIGHT_WALL) {
    steeringAngle = -steeringAngle;
  }
  
  // Drive with dynamic speed
  driveForward(targetSpeed);  // Use driveForward for correct direction
  setSteering(steeringAngle);
  systemState.currentSpeed = targetSpeed;
  
  // Show status (only when speed changed)
  static int16_t lastDisplayedSpeed = 0;
  if (abs(targetSpeed - lastDisplayedSpeed) > 20) {
    char status[32];
    sprintf(status, "V=%d Dist=%d", targetSpeed, wall.avgDistance);
    showDriveStatus(status);
    lastDisplayedSpeed = targetSpeed;
  }
}

// ============================================================================
// MAIN FUNCTION - STATE MACHINE
// ============================================================================

void autonomousDrive(const int16_t* distances) {
  switch (systemState.state) {
    
    case STATE_INIT:
      // Activate all sensors for initialization
      activateAllSensors();
      delay(100);  // Wait briefly until sensors active
      // Initialize with valid data on next call
      if (distances[0] != 0 || distances[1] != 0) {
        initializeDriving(distances);
      }
      break;
      
    case STATE_READY:
      // Wait for start (goes directly to DRIVING in initializeDriving)
      break;
      
    case STATE_DRIVING:
      // Normal driving
      executeDriving(distances);
      break;
      
    case STATE_OBSTACLE:
      // Obstacle detected - check if enough distance to continue
      {
        int16_t targetSpeed = calculateDynamicSpeed(distances);
        if (targetSpeed > 0) {
          // Enough distance -> continue
          resetSteeringIntegral();  // Reset PI controller
          systemState.state = STATE_DRIVING;
          showDriveStatus("Continue");
        } else {
          // Still too close -> remain stopped
          stopVehicle();
          centerSteering();
        }
      }
      break;
      
    case STATE_ERROR:
      // Error - system stopped
      // Only recoverable by reset
      break;
  }
}

// ============================================================================
// END
// ============================================================================