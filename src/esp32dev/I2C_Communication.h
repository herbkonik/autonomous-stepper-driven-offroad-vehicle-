/*
 * MC2 - I2C Communication
 * I2C Master for MC1 Control
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Setup
void setupI2C();

// Low-Level Register Access
void sendI2CRegister(uint8_t reg, int16_t value);

// High-Level Commands
void setSpeed(int16_t speed);
void setDirection(bool forward);
void setSteering(int16_t angle);
void sendCommand(uint8_t cmd);

// Driving Functions
void driveForward(int16_t speed);
void driveBackward(int16_t speed);
void stopVehicle();
void emergencyStop();
void centerSteering();
