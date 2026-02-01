/*
 * MC1 - 4-Wheel Stepper Car - Antrieb & Lenkung
 *
 * Hardware:
 * - LGT8F328P LQFP48 MiniEVB (16MHz, 3.3V)
 * - 4× NEMA17 Stepper mit MKS Servo42C (FOC)
 * - 4× MG946R Servos (Lenkung)
 *
 * Funktionen:
 * - Timer-gesteuerte Motorsteuerung (non-blocking)
 * - Rampen für Motoren (sanftes Beschleunigen/Bremsen)
 * - Rampen für Servos (sanftes Lenken)
 * - Sprechende Namen für alle Komponenten
 * - High-Level Funktionen für Fahrmanöver
 * - I2C Slave Interface mit Register-System (für MC2 Kommunikation)
 *
 * Author: Herbert Kozuschnik
 * Date: 2026-01-24
 * Version: 0.3 (I2C Slave - Produktiv)
 */

#include <Arduino.h>
#include <ServoTimer2.h>
#include <Wire.h>
#include "Functions.h"

// ============================================================================
// KONSTANTEN & PIN-DEFINITIONEN
// ============================================================================

// Motor IDs (sprechende Namen)
#define MOTOR_FRONT_LEFT 0  // Step=3, Dir=4, Enable=31
#define MOTOR_FRONT_RIGHT 1 // Step=34, Dir=2, Enable=33
#define MOTOR_REAR_LEFT 2   // Step=9, Dir=10, Enable=11
#define MOTOR_REAR_RIGHT 3  // Step=21, Dir=25, Enable=20

// Servo IDs - MIT DUMMY wegen ServoTimer2 Bug!
#define SERVO_DUMMY 0       // NICHT BENUTZEN! (ServoTimer2 Bug Workaround)
#define SERVO_FRONT_LEFT 1  // Pin=5
#define SERVO_FRONT_RIGHT 2 // Pin=24
#define SERVO_REAR_LEFT 3   // Pin=12
#define SERVO_REAR_RIGHT 4  // Pin=26

// Motor Pin Definitionen
const byte motorStepPin[4] = {3, 34, 9, 21};
const byte motorDirPin[4] = {4, 2, 10, 25};
const byte motorEnablePin[4] = {31, 33, 11, 20};

// Servo Pin Definitionen - MIT DUMMY PIN!
const byte servoPin[5] = {99, 5, 24, 12, 26}; // Pin 99 = Dummy, wird nie benutzt

// Servo Kalibrierung (in Mikrosekunden)
constexpr int SERVO_CENTER = 1500; // Geradeaus
constexpr int SERVO_MIN = 600;     // Max links (-90°) für 270° Servos
constexpr int SERVO_MAX = 2400;    // Max rechts (+90°) für 270° Servos

// Motor Parameter
constexpr int MAX_MOTOR_SPEED = 800;              // Steps/s
constexpr int MOTOR_ACCEL_RATE = 50;              // Steps/s² (Beschleunigung)
constexpr unsigned long MOTOR_RAMP_INTERVAL = 50; // ms zwischen Rampenschritten

// Servo Parameter
constexpr int SERVO_RAMP_STEP = 1;                // Grad pro Schritt
constexpr unsigned long SERVO_RAMP_INTERVAL = 20; // ms zwischen Schritten

// Fahrzeugparameter
constexpr int AXLE_TRACK = 370;    // mm (Abstand Radmitte - Radmitte)
constexpr int AXLE_DISTANCE = 450; // mm (Abstand Vorder- Hinterachse)

// I2C Konfiguration
constexpr byte I2C_ADDRESS = 0x10;

// I2C Register-Adressen (von MC2 beschreibbar)
constexpr byte REG_SPEED = 0x01;    // Speed Register (-800 bis +800 Steps/s)
constexpr byte REG_STEERING = 0x02; // Steering Register (-90 bis +90 Grad)
constexpr byte REG_COMMAND = 0x03;  // Command Register (spezielle Befehle)

// Update Flags (welche Register wurden aktualisiert?)
constexpr uint8_t FLAG_SPEED_UPDATED = 0x01;
constexpr uint8_t FLAG_STEERING_UPDATED = 0x02;
constexpr uint8_t FLAG_COMMAND_UPDATED = 0x04;

// Command Codes für REG_COMMAND
constexpr uint8_t CMD_STOP = 0x01;            // Normaler Stopp (mit Rampe)
constexpr uint8_t CMD_EMERGENCY_STOP = 0xFF;  // Notaus (sofort)
constexpr uint8_t CMD_CENTER_STEERING = 0x02; // Lenkung zentrieren

// ============================================================================
// DATENSTRUKTUREN
// ============================================================================

// Motor Struktur
struct Motor {
  byte stepPin;
  byte dirPin;
  byte enablePin;
  volatile bool running;
  volatile bool stepState;
  volatile unsigned long stepCount;
  uint16_t stepDelay; // Aktuelle Geschwindigkeit (µs)
  unsigned long lastStepTime;

  // Rampen-Parameter
  int currentSpeed; // Aktuelle Geschwindigkeit (Steps/s)
  int targetSpeed;  // Ziel-Geschwindigkeit (Steps/s)
  unsigned long lastRampTime;
};

// Servo Struktur
struct ServoControl {
  ServoTimer2 servo;
  int currentAngle; // Aktueller Winkel (Grad, -90 bis +90)
  int targetAngle;  // Ziel-Winkel (Grad, -90 bis +90)
  unsigned long lastRampTime;
};

// ============================================================================
// GLOBALE VARIABLEN
// ============================================================================

Motor motors[4];
ServoControl servos[5]; // 5 statt 4 wegen Dummy-Servo

// I2C Register (von MC2 via I2C beschreibbar)
volatile int16_t regSpeed = 0;       // Ziel-Geschwindigkeit (-800 bis +800)
volatile int16_t regSteering = 0;    // Ziel-Lenkwinkel (-90 bis +90)
volatile uint8_t regCommand = 0;     // Spezielle Befehle
volatile uint8_t regUpdateFlags = 0; // Flags welche Register aktualisiert wurden

// ============================================================================
// TIMER1 INTERRUPT (Motorsteuerung)
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
// SETUP FUNKTIONEN
// ============================================================================

void setupTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = (F_CPU / 20000UL) - 1; // 20kHz Interrupt
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
    digitalWrite(motors[i].enablePin, LOW); // Enable LOW für Servo42C
  }
}

void setupServos() {
  // BEGINNE BEI INDEX 1 wegen ServoTimer2 Bug (Index 0 funktioniert nicht)
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
// LOW-LEVEL MOTOR FUNKTIONEN
// ============================================================================

void setMotorSpeed(byte motorID, int stepsPerSec) {
  if (motorID >= 4)
    return;

  // Begrenzung
  stepsPerSec = constrain(stepsPerSec, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motors[motorID].targetSpeed = stepsPerSec;
}

void updateMotorSpeed(byte motorID) {
  if (motorID >= 4)
    return;

  Motor &m = motors[motorID];

  // Rampen-Update (nur alle MOTOR_RAMP_INTERVAL ms)
  unsigned long currentTime = millis();
  if (currentTime - m.lastRampTime < MOTOR_RAMP_INTERVAL) {
    return;
  }
  m.lastRampTime = currentTime;

  // Beschleunigung/Verzögerung
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

  // Geschwindigkeit anwenden
  if (m.currentSpeed == 0) {
    m.running = false;
    digitalWrite(m.stepPin, LOW);
  } else {
    // Richtung setzen - MIT INVERTIERUNG FÜR LINKE SEITE
    bool direction = m.currentSpeed > 0;

    // Invertiere linke Seite (sonst laufen links/rechts gegenläufig)
    if (motorID == MOTOR_FRONT_LEFT || motorID == MOTOR_REAR_LEFT) {
      direction = !direction;
    }

    digitalWrite(m.dirPin, direction ? HIGH : LOW);

    // Step Delay berechnen
    uint16_t absSpeed = abs(m.currentSpeed);
    m.stepDelay = 500000UL / absSpeed; // Mikrosekunden
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
// LOW-LEVEL SERVO FUNKTIONEN
// ============================================================================

void setServoAngle(byte servoID, int angle) {
  if (servoID >= 5 || servoID == 0)
    return; // Index 0 = Dummy, überspringen

    // Begrenzung
    angle = constrain(angle, -90, 90);
  servos[servoID].targetAngle = angle;
}

void setServoAngle(byte servoID, float angle) {
  setServoAngle(servoID, (int)angle);
}

void updateServoAngle(byte servoID) {
  if (servoID >= 5 || servoID == 0)
    return; // Index 0 = Dummy, überspringen

    ServoControl &s = servos[servoID];

  // Rampen-Update
  unsigned long currentTime = millis();
  if (currentTime - s.lastRampTime < SERVO_RAMP_INTERVAL) {
    return;
  }
  s.lastRampTime = currentTime;

  // Winkel anpassen
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

  // Winkel in Mikrosekunden umrechnen
  int pulseWidth = SERVO_CENTER + map(s.currentAngle, -90, 90, -900, 900);
  pulseWidth = constrain(pulseWidth, SERVO_MIN, SERVO_MAX);
  s.servo.write(pulseWidth);
}

// ============================================================================
// HIGH-LEVEL FUNKTIONEN (Gruppen)
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
  for (byte i = 1; i < 5; i++) { // Starte bei 1 wegen Dummy
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
// HIGH-LEVEL FUNKTIONEN (Fahrmanöver)
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
  // Vorne: nach links (negativ)
  setServoAngle(SERVO_FRONT_LEFT, -abs(angle));
  setServoAngle(SERVO_FRONT_RIGHT, -abs(angle));

  // Hinten: nach rechts (positiv, invertiert!)
  setServoAngle(SERVO_REAR_LEFT, abs(angle));
  setServoAngle(SERVO_REAR_RIGHT, abs(angle));
}

void steerRight(float angle) { steerRight((int)angle); }

void steerLeft(int angle) {
  // Vorne: nach rechts (positiv)
  setServoAngle(SERVO_FRONT_LEFT, abs(angle));
  setServoAngle(SERVO_FRONT_RIGHT, abs(angle));

  // Hinten: nach links (negativ, invertiert!)
  setServoAngle(SERVO_REAR_LEFT, -abs(angle));
  setServoAngle(SERVO_REAR_RIGHT, -abs(angle));
}

void steerLeft(float angle) { steerLeft((int)angle); }

void stop() { stopAllMotors(); }

// ============================================================================
// I2C KOMMUNIKATION (Register-basiert)
// ============================================================================

void i2cReceiveEvent(int numBytes) {
  if (numBytes < 3)
    return; // Mindestens: Register + 2 Bytes Wert

    byte reg = Wire.read(); // Register-Adresse

    // Wert lesen (16-bit, Big Endian)
    int16_t value = Wire.read() << 8;
  value |= Wire.read();

  // In entsprechendes Register schreiben
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
  // Speed Register verarbeiten
  if (regUpdateFlags & FLAG_SPEED_UPDATED) {
    setAllMotorsSpeed(regSpeed);
    regUpdateFlags &= ~FLAG_SPEED_UPDATED;
  }

  // Steering Register verarbeiten
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

  // Command Register verarbeiten
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
// UPDATE FUNKTIONEN
// ============================================================================

void updateAllMotors() {
  for (byte i = 0; i < 4; i++) {
    updateMotorSpeed(i);
  }
}

void updateAllServos() {
  for (byte i = 1; i < 5; i++) { // Starte bei 1 wegen Dummy
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

  delay(2000);  // Startpause
}

void loop() {
  // I2C Register verarbeiten
  processRegisters();

  // Motoren & Servos aktualisieren (Rampen)
  updateAllMotors();
  updateAllServos();

  delay(1);
}
