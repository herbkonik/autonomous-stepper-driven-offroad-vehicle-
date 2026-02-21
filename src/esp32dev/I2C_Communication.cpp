/*
 * MC2 - I2C Implementation
 * I2C Master for MC1 Control
 * 
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 1.0
 */

#include "I2C_Communication.h"

// ============================================================================
// SETUP
// ============================================================================

void setupI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK);
}

// ============================================================================
// LOW-LEVEL REGISTER ACCESS
// ============================================================================

void sendI2CRegister(uint8_t reg, int16_t value) {
  Wire.beginTransmission(MC1_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);  // High Byte
  Wire.write(value & 0xFF);         // Low Byte
  Wire.endTransmission();
}

// ============================================================================
// HIGH-LEVEL COMMANDS 
// ============================================================================

void setSpeed(int16_t speed) {
  sendI2CRegister(REG_SPEED, speed);
}

void setDirection(bool forward) {
  sendI2CRegister(REG_DIRECTION, forward ? 1 : 0);
}

void setSteering(int16_t angle) {
  sendI2CRegister(REG_STEERING, angle);
}

void sendCommand(uint8_t cmd) {
  sendI2CRegister(REG_COMMAND, cmd);
}

// ============================================================================
// DRIVING FUNCTIONS
// ============================================================================

void driveForward(int16_t speed) {
  setSpeed(speed);      // Positive speed
  setDirection(true);   // true = forward
}

void driveBackward(int16_t speed) {
  setSpeed(speed);      // Positive speed
  setDirection(false);  // false = backward
}

void stopVehicle() {
  sendCommand(CMD_STOP);
}

void emergencyStop() {
  sendCommand(CMD_EMERGENCY_STOP);
}

void centerSteering() {
  sendCommand(CMD_CENTER_STEERING);
}
