/*
 * MC2 - Display Management
 * TFT ST7789VW (240×320)
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

// Setup
void setupDisplay();

// Display Updates
void updateSensorDisplay(const int16_t* distances);
void showMeasuringStatus(uint8_t currentPass);
void showDriveStatus(const char* status);
void showInitStatus(const char* message);
