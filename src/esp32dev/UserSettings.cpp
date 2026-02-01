/*
 * MC2 - User Settings (Definitions)
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#include "UserSettings.h"

// ============================================================================
// GLOBAL SETTINGS (changeable during runtime)
// ============================================================================

// Target distance to wall (in mm)
int16_t targetWallDistance = 200;

// Driving speed (Steps/s)
int16_t driveSpeed = 400;

// Maximum steering angle (degrees)
int16_t maxSteeringAngle = 25;
