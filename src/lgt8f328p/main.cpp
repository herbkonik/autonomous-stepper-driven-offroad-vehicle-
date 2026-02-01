/*
 * MC1 - 4-Wheel Stepper Car - Drive & Steering
 *
 * Hardware:
 * - LGT8F328P LQFP48 MiniEVB (16MHz, 3.3V)
 * - 4× NEMA17 Stepper with MKS Servo42C (FOC)
 * - 4× MG946R Servos (Steering)
 *
 * Features:
 * - Timer-controlled motor management (non-blocking)
 * - Ramps for motors (smooth acceleration/deceleration)
 * - Ramps for servos (smooth steering)
 * - Descriptive names for all components
 * - High-level functions for driving maneuvers
 * - I2C Slave interface with register system (for MC2 communication)
 *
 * Author: Herbert Kozuschnik
 * License: GPLv3 ( https://www.gnu.org/licenses/gpl-3.0.html.en )
 * Date: 2026-01-24
 * Version: 0.3 (I2C Slave - Production)
 */

#include <Arduino.h>
#include <ServoTimer2.h>
#include <Wire.h>
#include "Functions.h"

// ============================================================================
// CONSTANTS & PIN DEFINITIONS
// ============================================================================

// Motor IDs (descriptive names)
#define MOTOR_FRONT_LEFT 0  // Step=3, Dir=4, Enable=31
#define MOTOR_FRONT_RIGHT 1 // Step=34, Dir=2, Enable=33
#define MOTOR_REAR_LEFT 2   // Step=9, Dir=10, Enable=11
#define MOTOR_REAR_RIGHT 3  // Step=21, Dir=25, Enable=20

// Servo IDs - WITH DUMMY due to ServoTimer2 Bug!
#define SERVO_DUMMY 0       // DO NOT USE! (ServoTimer2 Bug Workaround)
#define SERVO_FRONT_LEFT 1  // Pin=5
#define SERVO_FRONT_RIGHT 2 // Pin=24
#define SERVO_REAR_LEFT 3   // Pin=12
#define SERVO_REAR_RIGHT 4  // Pin=26

// Motor Pin Definitions
const byte motorStepPin[4] = {3, 34, 9, 21};
const byte motorDirPin[4] = {4, 2, 10, 25};
const byte motorEnablePin[4] = {31, 33, 11, 20};

// Servo Pin Definitions - WITH DUMMY PIN!
const byte servoPin[5] = {99, 5, 24, 12, 26}; // Pin 99 = Dummy, never used

// Servo Calibration (in microseconds)
constexpr int SERVO_CENTER = 1500; // Straight ahead
constexpr int SERVO_MIN = 600;     // Max left (-90°) for 270° servos
constexpr int SERVO_MAX = 2400;    // Max right (+90°) for 270° servos

// Motor Parameters
constexpr int MAX_MOTOR_SPEED = 800;              // Steps/s
constexpr int MOTOR_ACCEL_RATE = 50;              // Steps/s² (acceleration)
constexpr unsigned long MOTOR_RAMP_INTERVAL = 50; // ms between ramp steps

// Servo Parameters
constexpr int SERVO_RAMP_STEP = 1;                // Degrees per step
constexpr unsigned long SERVO_RAMP_INTERVAL = 20; // ms between steps

// Vehicle Parameters
constexpr int AXLE_TRACK = 370;    // mm (distance wheel center - wheel center)
constexpr int AXLE_DISTANCE = 450; // mm (distance front - rear axle)

// I2C Configuration
constexpr byte I2C_ADDRESS = 0x10;

// I2C Register Addresses (writable by MC2)
constexpr byte REG_SPEED = 0x01;    // Speed Register (-800 to +800 Steps/s)
constexpr byte REG_STEERING = 0x02; // Steering Register (-90 to +90 degrees)
constexpr byte REG_COMMAND = 0x03;  // Command Register (special commands)

// Update Flags (which registers were updated?)
constexpr uint8_t FLAG_SPEED_UPDATED = 0x01;
constexpr uint8_t FLAG_STEERING_UPDATED = 0x02;
constexpr uint8_t FLAG_COMMAND_UPDATED = 0x04;

// Command Codes for REG_COMMAND
constexpr uint8_t CMD_STOP = 0x01;            // Normal stop (with ramp)
constexpr uint8_t CMD_EMERGENCY_STOP = 0xFF;  // Emergency stop (immediate)
constexpr uint8_t CMD_CENTER_STEERING = 0x02; // Center steering

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Motor Structure
struct Motor {
  byte stepPin;
  byte dirPin;
  byte enablePin;
  volatile bool running;
  volatile bool stepState;
  volatile unsigned long stepCount;
  uint16_t stepDelay; // Current speed (µs)
  unsigned long lastStepTime;

  // Ramp Parameters
  int currentSpeed; // Current speed (Steps/s)
  int targetSpeed;  // Target speed (Steps/s)
  unsigned long lastRampTime;
};

// Servo Structure
struct ServoControl {
  ServoTimer2 servo;
  int currentAngle; // Current angle (degrees, -90 to +90)
  int targetAngle;  // Target angle (degrees, -90 to +90)
  unsigned long lastRampTime;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Motor motors[4];
ServoControl servos[5]; // 5 instead of 4 due to dummy servo

// I2C Registers (writable by MC2 via I2C)
volatile int16_t regSpeed = 0;       // Target speed (-800 to +800)
volatile int16_t regSteering = 0;    // Target steering angle (-90 to +90)
volatile uint8_t regCommand = 0;     // Special commands
volatile uint8_t regUpdateFlags = 0; // Flags which registers were updated

// ============================================================================
// TIMER1 INTERRUPT (Motor Control)
// ============================================================================

ISR(TIMER1_COMPA_vect) {
  unsigned long currentMicros = micros();

  for (byte i = 0; i < 4; i++) {
    if (motors[i].running && motors[i].stepDelay > 0) {
      if (currentMicros - motors[i].lastStepTime >= motors[i].stepDelay) {
        motors[i].lastStepTime = currentMicros;
        motors[i].stepState = !motors[i].stepState;
        digitalWrite(motors[i].stepPin, motors[i].stepState);

        if (motors[i].stepState) {
          motors[i].stepCount++;
        }
      }
    }
  }
}

// ============================================================================
// SETUP FUNCTIONS
// ============================================================================

void setupTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = (F_CPU / 20000UL) - 1; // 20kHz interrupt
  TCCR1B |= (1 << WGM12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

void setupMotors() {
  for (byte i = 0; i < 4; i++) {
    motors[i].stepPin = motorStepPin[i];
    motors[i].dirPin = motorDirPin[i];
    motors[i].enablePin = motorEnablePin[i];
    motors[i].running = false;
    motors[i].stepState = LOW;
    motors[i].stepCount = 0;
    motors[i].stepDelay = 1000;
    motors[i].lastStepTime = 0;
    motors[i].currentSpeed = 0;
    motors[i].targetSpeed = 0;
    motors[i].lastRampTime = 0;

    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    pinMode(motors[i].enablePin, OUTPUT);

    digitalWrite(motors[i].stepPin, LOW);
    digitalWrite(motors[i].dirPin, LOW);
    digitalWrite(motors[i].enablePin, LOW); // Enable LOW for Servo42C
  }
}

void setupServos() {
  // START AT INDEX 1 due to ServoTimer2 bug (index 0 doesn't work)
  for (byte i = 1; i < 5; i++) {
    servos[i].servo.attach(servoPin[i]);
    servos[i].currentAngle = 0;
    servos[i].targetAngle = 0;
    servos[i].lastRampTime = 0;
    servos[i].servo.write(SERVO_CENTER);
  }
}

void setupI2C() {
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(i2cReceiveEvent);
}

// ============================================================================
// LOW-LEVEL MOTOR FUNCTIONS
// ============================================================================

void setMotorSpeed(byte motorID, int stepsPerSec) {
  if (motorID >= 4)
    return;

  // Limit speed
  stepsPerSec = constrain(stepsPerSec, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motors[motorID].targetSpeed = stepsPerSec;
}

void updateMotorSpeed(byte motorID) {
  if (motorID >= 4)
    return;

  Motor &m = motors[motorID];

  // Ramp update (only every MOTOR_RAMP_INTERVAL ms)
  unsigned long currentTime = millis();
  if (currentTime - m.lastRampTime < MOTOR_RAMP_INTERVAL) {
    return;
  }
  m.lastRampTime = currentTime;

  // Acceleration/Deceleration
  if (m.currentSpeed < m.targetSpeed) {
    m.currentSpeed += MOTOR_ACCEL_RATE;
    if (m.currentSpeed > m.targetSpeed) {
      m.currentSpeed = m.targetSpeed;
    }
  } else if (m.currentSpeed > m.targetSpeed) {
    m.currentSpeed -= MOTOR_ACCEL_RATE;
    if (m.currentSpeed < m.targetSpeed) {
      m.currentSpeed = m.targetSpeed;
    }
  }

  // Apply speed
  if (m.currentSpeed == 0) {
    m.running = false;
    digitalWrite(m.stepPin, LOW);
  } else {
    // Set direction - WITH INVERSION FOR LEFT SIDE
    bool direction = m.currentSpeed > 0;

    // Invert left side (otherwise left/right run opposite)
    if (motorID == MOTOR_FRONT_LEFT || motorID == MOTOR_REAR_LEFT) {
      direction = !direction;
    }

    digitalWrite(m.dirPin, direction ? HIGH : LOW);

    // Calculate step delay
    uint16_t absSpeed = abs(m.currentSpeed);
    m.stepDelay = 500000UL / absSpeed; // Microseconds
    m.running = true;
  }
}

void stopMotor(byte motorID) {
  if (motorID >= 4)
    return;
  setMotorSpeed(motorID, 0);
}

void emergencyStopMotor(byte motorID) {
  if (motorID >= 4)
    return;
  motors[motorID].currentSpeed = 0;
  motors[motorID].targetSpeed = 0;
  motors[motorID].running = false;
  digitalWrite(motors[motorID].stepPin, LOW);
}

// ============================================================================
// LOW-LEVEL SERVO FUNCTIONS
// ============================================================================

void setServoAngle(byte servoID, int angle) {
  if (servoID >= 5 || servoID == 0)
    return; // Index 0 = Dummy, skip

  // Limit angle
  angle = constrain(angle, -90, 90);
  servos[servoID].targetAngle = angle;
}

void setServoAngle(byte servoID, float angle) {
  setServoAngle(servoID, (int)angle);
}

void updateServoAngle(byte servoID) {
  if (servoID >= 5 || servoID == 0)
    return; // Index 0 = Dummy, skip

  ServoControl &s = servos[servoID];

  // Ramp update
  unsigned long currentTime = millis();
  if (currentTime - s.lastRampTime < SERVO_RAMP_INTERVAL) {
    return;
  }
  s.lastRampTime = currentTime;

  // Adjust angle
  if (s.currentAngle < s.targetAngle) {
    s.currentAngle += SERVO_RAMP_STEP;
    if (s.currentAngle > s.targetAngle) {
      s.currentAngle = s.targetAngle;
    }
  } else if (s.currentAngle > s.targetAngle) {
    s.currentAngle -= SERVO_RAMP_STEP;
    if (s.currentAngle < s.targetAngle) {
      s.currentAngle = s.targetAngle;
    }
  }

  // Convert angle to microseconds
  int pulseWidth = SERVO_CENTER + map(s.currentAngle, -90, 90, -900, 900);
  pulseWidth = constrain(pulseWidth, SERVO_MIN, SERVO_MAX);
  s.servo.write(pulseWidth);
}

// ============================================================================
// HIGH-LEVEL FUNCTIONS (Groups)
// ============================================================================

void setAllMotorsSpeed(int speed) {
  for (byte i = 0; i < 4; i++) {
    setMotorSpeed(i, speed);
  }
}

void setLeftSideSpeed(int speed) {
  setMotorSpeed(MOTOR_FRONT_LEFT, speed);
  setMotorSpeed(MOTOR_REAR_LEFT, speed);
}

void setRightSideSpeed(int speed) {
  setMotorSpeed(MOTOR_FRONT_RIGHT, speed);
  setMotorSpeed(MOTOR_REAR_RIGHT, speed);
}

void setAllSteering(int angle) {
  for (byte i = 1; i < 5; i++) { // Start at 1 due to dummy
    setServoAngle(i, angle);
  }
}

void setAllSteering(float angle) {
  setAllSteering((int)angle);
}

void stopAllMotors() {
  for (byte i = 0; i < 4; i++) {
    stopMotor(i);
  }
}

void emergencyStopAll() {
  for (byte i = 0; i < 4; i++) {
    emergencyStopMotor(i);
  }
}

void centerAllSteering() { setAllSteering(0); }

// ============================================================================
// HIGH-LEVEL FUNCTIONS (Driving Maneuvers)
// ============================================================================

void driveForward(int speed) { setAllMotorsSpeed(-speed); }

void driveBackward(int speed) { setAllMotorsSpeed(speed); }

void turnLeft(int baseSpeed, int speedDifference) {
  setLeftSideSpeed(baseSpeed - speedDifference);
  setRightSideSpeed(baseSpeed + speedDifference);
}

void turnRight(int baseSpeed, int speedDifference) {
  setLeftSideSpeed(baseSpeed + speedDifference);
  setRightSideSpeed(baseSpeed - speedDifference);
}

void steerRight(int angle) {
  // Front: to left (negative)
  setServoAngle(SERVO_FRONT_LEFT, -abs(angle));
  setServoAngle(SERVO_FRONT_RIGHT, -abs(angle));

  // Rear: to right (positive, inverted!)
  setServoAngle(SERVO_REAR_LEFT, abs(angle));
  setServoAngle(SERVO_REAR_RIGHT, abs(angle));
}

void steerRight(float angle) { steerRight((int)angle); }

void steerLeft(int angle) {
  // Front: to right (positive)
  setServoAngle(SERVO_FRONT_LEFT, abs(angle));
  setServoAngle(SERVO_FRONT_RIGHT, abs(angle));

  // Rear: to left (negative, inverted!)
  setServoAngle(SERVO_REAR_LEFT, -abs(angle));
  setServoAngle(SERVO_REAR_RIGHT, -abs(angle));
}

void steerLeft(float angle) { steerLeft((int)angle); }

void stop() { stopAllMotors(); }

// ============================================================================
// I2C COMMUNICATION (Register-based)
// ============================================================================

void i2cReceiveEvent(int numBytes) {
  if (numBytes < 3)
    return; // Minimum: Register + 2 bytes value

  byte reg = Wire.read(); // Register address

  // Read value (16-bit, Big Endian)
  int16_t value = Wire.read() << 8;
  value |= Wire.read();

  // Write to corresponding register
  switch (reg) {
    case REG_SPEED:
      regSpeed = constrain(value, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      regUpdateFlags |= FLAG_SPEED_UPDATED;
      break;

    case REG_STEERING:
      regSteering = constrain(value, -90, 90);
      regUpdateFlags |= FLAG_STEERING_UPDATED;
      break;

    case REG_COMMAND:
      regCommand = value & 0xFF;
      regUpdateFlags |= FLAG_COMMAND_UPDATED;
      break;
  }
}

void processRegisters() {
  // Process Speed Register
  if (regUpdateFlags & FLAG_SPEED_UPDATED) {
    setAllMotorsSpeed(regSpeed);
    regUpdateFlags &= ~FLAG_SPEED_UPDATED;
  }

  // Process Steering Register
  if (regUpdateFlags & FLAG_STEERING_UPDATED) {
    if (regSteering > 0) {
      steerRight(regSteering);
    } else if (regSteering < 0) {
      steerLeft(-regSteering);
    } else {
      centerAllSteering();
    }
    regUpdateFlags &= ~FLAG_STEERING_UPDATED;
  }

  // Process Command Register
  if (regUpdateFlags & FLAG_COMMAND_UPDATED) {
    switch (regCommand) {
      case CMD_STOP:
        stop();
        break;

      case CMD_EMERGENCY_STOP:
        emergencyStopAll();
        break;

      case CMD_CENTER_STEERING:
        centerAllSteering();
        break;
    }
    regUpdateFlags &= ~FLAG_COMMAND_UPDATED;
  }
}

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================

void updateAllMotors() {
  for (byte i = 0; i < 4; i++) {
    updateMotorSpeed(i);
  }
}

void updateAllServos() {
  for (byte i = 1; i < 5; i++) { // Start at 1 due to dummy
    updateServoAngle(i);
  }
}

// ============================================================================
// SETUP & LOOP
// ============================================================================

void setup() {
  setupMotors();
  setupTimer();
  setupServos();
  setupI2C();

  delay(2000);  // Startup pause
}

void loop() {
  // Process I2C registers
  processRegisters();

  // Update motors & servos (ramps)
  updateAllMotors();
  updateAllServos();

  delay(1);
}