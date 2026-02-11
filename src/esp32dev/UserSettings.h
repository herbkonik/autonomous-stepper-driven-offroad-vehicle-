/*
 * MC2 - User Settings
 * All adjustable parameters for wall-following
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#pragma once

#include <Arduino.h>

// ============================================================================
// VEHICLE GEOMETRY
// ============================================================================

// Distance between front and rear side sensors (in mm)
constexpr int16_t SIDE_SENSOR_DISTANCE = 600;  // 600mm between sensors

// ============================================================================
// WALL-FOLLOWING PARAMETERS
// ============================================================================

// Target distance to wall (in mm)
extern int16_t targetWallDistance;

// Driving speed (Steps/s)
extern int16_t driveSpeed;

// Maximum steering angle (degrees)
extern int16_t maxSteeringAngle;

// ============================================================================
// DISTANCE CONTROL
// ============================================================================

// Tolerance for wall distance (in mm)
// Correction only if deviation > tolerance
constexpr int16_t DISTANCE_TOLERANCE = 20;

// Proportional gain for distance control
constexpr float DISTANCE_P_GAIN = 0.15;

// ============================================================================
// ANGLE CONTROL (PI Controller)
// ============================================================================

// Minimum angle to wall that triggers correction (in degrees)
// Positive value = vehicle points toward wall
// Negative value = vehicle points away from wall
constexpr float MIN_ANGLE_CORRECTION = 2.0;  // ±2° dead zone

// P-gain: Gain for angle-based steering
constexpr float ANGLE_P_GAIN = 1.0;  // Degrees steering per degree angle

// I-gain: Gain for error integral (accumulated angle error)
constexpr float ANGLE_I_GAIN = 0.5;  // Degrees steering per (degree × second)

// Anti-Windup: Maximum I-component (prevents overshoot)
constexpr float ANGLE_I_LIMIT = 8.0;  // Max ±8° steering from I-component

// ============================================================================
// WEIGHTING DISTANCE vs. ANGLE
// ============================================================================

// How much distance vs. angle is weighted (0.0 - 1.0)
// 0.0 = only distance, 1.0 = only angle, 0.5 = equal
constexpr float ANGLE_WEIGHT = 0.3;  // 30% angle, 70% distance

// ============================================================================
// DYNAMIC SPEED CONTROL
// ============================================================================

// Thresholds for speed reduction (in mm)
constexpr int16_t SPEED_REDUCE_START = 1000;  // Start braking
constexpr int16_t SPEED_REDUCE_25 = 800;      // -25% speed
constexpr int16_t SPEED_REDUCE_50 = 600;      // -50% speed
constexpr int16_t SPEED_REDUCE_75 = 400;      // -75% speed
constexpr int16_t SPEED_STOP = 200;           // Full stop

// ============================================================================
// WALL DETECTION
// ============================================================================

// Maximum distance to detect wall (in mm)
constexpr int16_t WALL_DETECTION_MAX = 800;

// Minimum distance for wall detection (prevents false detection for very close objects)
constexpr int16_t WALL_DETECTION_MIN = 50;

// ============================================================================
// SENSOR OFFSETS (CALIBRATION)
// ============================================================================

// Adjust these values in Config.h
// SENSOR_OFFSETS[8] = { -20, -20, -20, ... };
