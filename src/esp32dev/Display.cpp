/*
 * MC2 - Display Implementation
 * TFT ST7789VW (240×320)
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#include "Display.h"

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

TFT_eSPI tft = TFT_eSPI();

// ============================================================================
// SETUP
// ============================================================================

void setupDisplay() {
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(COLOR_BG);

  tft.setTextSize(2);
  tft.setTextColor(COLOR_HEADER, COLOR_BG);
  tft.setCursor(10, 10);
  tft.println("MC2 Dual-Core");

  tft.drawFastHLine(0, 35, 240, COLOR_HEADER);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_US, COLOR_BG);
  tft.setCursor(10, 45);
  tft.print("Sensors+I2C+Auto");
}

// ============================================================================
// SENSOR DISPLAY
// ============================================================================

void updateSensorDisplay(const int16_t* distances) {
  struct SensorPos {
    uint8_t sensor;
    int16_t x;
    int16_t y;
  };

  SensorPos positions[8] = {
    {0,  10, 230},  // L-Side-Rear
    {1,  30, 270},  // L-Rear
    {2, 140, 270},  // R-Rear
    {3, 160, 230},  // R-Side-Rear
    {4, 160, 110},  // R-Side-Front
    {5, 140,  70},  // R-Front
    {6,  10, 110},  // L-Side-Front
    {7,  30,  70}   // L-Front
  };

  for (uint8_t i = 0; i < 8; i++) {
    uint8_t s = positions[i].sensor;
    int16_t x = positions[i].x;
    int16_t y = positions[i].y;

    tft.fillRect(x, y, 72, 24, COLOR_BG);
    tft.setTextSize(3);
    tft.setCursor(x, y);

    if (distances[s] == -2) {
      // Sensor inactive
      tft.setTextColor(COLOR_INACTIVE, COLOR_BG);
      tft.print(" OFF");
    } else if (distances[s] == -1) {
      // Measurement error
      tft.setTextColor(COLOR_ERROR, COLOR_BG);
      tft.print(" ERR");
    } else {
      // Valid measurement
      if (distances[s] < 100) {
        tft.setTextColor(COLOR_WARNING, COLOR_BG);
      } else {
        tft.setTextColor(COLOR_US, COLOR_BG);
      }
      tft.printf("%4d", distances[s]);
    }
  }

  // Footer with status
  tft.fillRect(0, 300, 240, 20, COLOR_BG);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_OK, COLOR_BG);
  tft.setCursor(10, 305);
  tft.print("Dual-Core Active");
}

// ============================================================================
// STATUS DISPLAYS
// ============================================================================

void showMeasuringStatus(uint8_t currentPass) {
  tft.fillRect(0, 300, 240, 20, COLOR_BG);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_WARNING, COLOR_BG);
  tft.setCursor(10, 305);
  tft.printf("Measuring %d/%d...", currentPass + 1, NUM_SAMPLES);
}

void showDriveStatus(const char* status) {
  tft.fillRect(0, 300, 240, 20, COLOR_BG);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_HEADER, COLOR_BG);
  tft.setCursor(10, 305);
  tft.print(status);
}

void showInitStatus(const char* message) {
  tft.fillScreen(COLOR_BG);
  tft.setTextSize(2);
  tft.setTextColor(COLOR_TEXT, COLOR_BG);
  tft.setCursor(10, 10);
  tft.println(message);
}
