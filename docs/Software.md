# Software Architecture

## Overview

The firmware spans two microcontrollers with distinct responsibilities:

- **MC1 (LGT8F328P):** Real-time motor control with timer interrupts

- **MC2 (ESP32):** Dual-core sensor processing and navigation logic

- **Communication:** I2C register-based protocol between MCUs

## System-Level Architecture

```
┌─────────────────────────────────────────────────────────┐    
│                 MC2 (ESP32 Dual-Core)                   │    
│                                                         │    
│  Core 0 (Main)              Core 1 (Sensors)            │    
│  ├─ Display Updates         ├─ HC-SR04 Measurements     │    
│  ├─ I2C Master              ├─ Median Filtering         │    
│  ├─ PI Controller           └─ Calibration              │    
│  ├─ State Machine                 │                     │    
│  └─ Navigation Logic              │                     │    
│         │                   ┌─────▼─────┐               │    
│         │                   │   Mutex   │               │    
│         │                   │  Protected│               │    
│         └─────────────────► │ SensorData│               │    
│                             └───────────┘               │    
└──────────────────┬──────────────────────────────────────┘    
                   │ I2C Commands (3.3V)    
                   │ [REG_SPEED, REG_STEERING, REG_COMMAND]    
┌──────────────────▼──────────────────────────────────────┐    
│              MC1 (LGT8F328P Single-Core)                │    
│                                                         │    
│  ┌──────────────────────────────────────────────┐       │    
│  │         Timer1 ISR (20kHz)                   │       │    
│  │  Generates step pulses for all 4 motors      │       │    
│  └──────────────────┬───────────────────────────┘       │    
│                     │                                   │    
│  Main Loop:         ▼                                   │    
│  ├─ Process I2C Registers                               │    
│  ├─ Update Motor Speeds (Ramps)                         │    
│  ├─ Update Servo Angles (Ramps)                         │    
│  └─ delay(1ms)                                          │    
│                                                         │    
└───────────┬─────────────────────┬───────────────────────┘    
            │                     │    
      [4× Steppers]          [4× Servos]    
       FOC Control           PWM Signals
```

## MC1: Motor Control Firmware

### File Structure

```
MC1_Firmware/    
├── MC1_main.cpp              # Main program + ISR    
└── MC1_Functions.h           # Function declarations
```

### Core Components

#### 1. Timer1 ISR - Step Pulse Generation

Runs at **20kHz** to generate smooth step pulses:

```
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
```

**Why 20kHz?**

- Fast enough for smooth motion (50µs resolution)

- Low overhead (~5% CPU at 16MHz)

- Non-blocking (ISR handles timing)

#### 2. Motor Ramp Control

Smooth acceleration/deceleration every 50ms:

```
void updateMotorSpeed(byte motorID) {    
  Motor &m = motors[motorID];    
    
  // Ramp update (only every 50ms)    
  if (millis() - m.lastRampTime < 50) return;    
  m.lastRampTime = millis();    
    
  // Accelerate or decelerate    
  if (m.currentSpeed < m.targetSpeed) {    
    m.currentSpeed += 50;  // +50 steps/s²    
  } else if (m.currentSpeed > m.targetSpeed) {    
    m.currentSpeed -= 50;  // -50 steps/s²    
  }    
    
  // Apply speed    
  if (m.currentSpeed != 0) {    
    m.stepDelay = 500000UL / abs(m.currentSpeed);  // µs per half-step    
    m.running = true;    
  } else {    
    m.running = false;    
  }    
}
```

**Result:** No jerky starts/stops, protects mechanical components

#### 3. Servo Ramp Control

Smooth steering transitions:

```
void updateServoAngle(byte servoID) {    
  ServoControl &s = servos[servoID];    
    
  // Ramp update (only every 20ms)    
  if (millis() - s.lastRampTime < 20) return;    
  s.lastRampTime = millis();    
    
  // Move 1° per step    
  if (s.currentAngle < s.targetAngle) {    
    s.currentAngle += 1;    
  } else if (s.currentAngle > s.targetAngle) {    
    s.currentAngle -= 1;    
  }    
    
  // Convert to PWM (1500µs center, ±900µs range)    
  int pulseWidth = 1500 + map(s.currentAngle, -90, 90, -900, 900);    
  s.servo.write(pulseWidth);    
}
```

**Result:** 1.8 seconds for full 90° → 0° → -90° sweep

#### 4. I2C Slave Interface

MC1 listens at address **0x10** for register writes:

```
void i2cReceiveEvent(int numBytes) {    
  if (numBytes < 3) return;  // Need: Reg + 2 bytes value    
    
  byte reg = Wire.read();    
  int16_t value = Wire.read() << 8;  // Big Endian    
  value |= Wire.read();    
    
  switch (reg) {    
    case 0x01:  // REG_SPEED    
      regSpeed = constrain(value, -800, 800);    
      regUpdateFlags |= FLAG_SPEED_UPDATED;    
      break;    
    
    case 0x02:  // REG_STEERING    
      regSteering = constrain(value, -90, 90);    
      regUpdateFlags |= FLAG_STEERING_UPDATED;    
      break;    
    
    case 0x03:  // REG_COMMAND    
      regCommand = value & 0xFF;    
      regUpdateFlags |= FLAG_COMMAND_UPDATED;    
      break;    
  }    
}
```

**Register Processing** (in main loop):

```
void processRegisters() {    
  if (regUpdateFlags & FLAG_SPEED_UPDATED) {    
    setAllMotorsSpeed(regSpeed);    
    regUpdateFlags &= ~FLAG_SPEED_UPDATED;    
  }    
    
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
    
  if (regUpdateFlags & FLAG_COMMAND_UPDATED) {    
    // Handle CMD_STOP, CMD_EMERGENCY_STOP, etc.    
    regUpdateFlags &= ~FLAG_COMMAND_UPDATED;    
  }    
}
```

### MC1 Main Loop

```
void loop() {    
  processRegisters();      // Handle I2C updates    
  updateAllMotors();       // Apply motor ramps    
  updateAllServos();       // Apply servo ramps    
  delay(1);                // 1ms cycle time    
}
```

**Performance:**

- Loop frequency: ~1kHz (1ms cycle)

- I2C latency: <1ms (interrupt-driven)

- Step generation: 20kHz (ISR, independent)

## MC2: Navigation Firmware

### File Structure

```
MC2_Firmware/    
├── main.cpp                  # Setup & main loop (Core 0)    
├── Config.h                  # Hardware definitions    
├── Sensors.cpp/h             # Sensor task (Core 1)    
├── Display.cpp/h             # TFT management    
├── I2C_Communication.cpp/h   # MC1 communication    
├── DrivingLogic.cpp/h        # State machine & PI control    
└── UserSettings.cpp/h        # Tunable parameters
```

### Core 0: Navigation Logic

#### Main Loop Flow

```
void loop() {    
  // 1. Check for new sensor data    
  if (isSensorDataReady()) {    
    getSensorData(currentDistances);  // Thread-safe read    
        
    // 2. Update display (rate-limited to 200ms)    
    if (millis() - lastDisplayUpdate >= 200) {    
      updateSensorDisplay(currentDistances);    
      lastDisplayUpdate = millis();    
    }    
  }    
      
  // 3. Execute navigation    
  autonomousDrive(currentDistances);    
      
  delay(10);  // Yield to FreeRTOS    
}
```

#### State Machine

```
                    ┌─────────────┐    
                    │ STATE_INIT  │    
                    └──────┬──────┘    
                           │ • Activate all 8 sensors    
                           │ • Detect wall (L/R/none)    
                           │ • Select wall to follow    
                           ▼    
                    ┌─────────────┐    
                    │ STATE_READY │    
                    └──────┬──────┘    
                           │ • Activate 4 sensors    
                           │ • Reset PI controller    
                           ▼    
                    ┌─────────────┐    
          ┌────────►│STATE_DRIVING│◄─────────┐    
          │         └──────┬──────┘          │    
          │                │                 │    
          │  Obstacle      │  Front sensors  │ Front sensors    
          │  cleared       ▼  < 200mm        │ > 200mm    
          │         ┌──────────────┐         │    
          └─────────│STATE_OBSTACLE├─────────┘    
                    └──────────────┘    
                        
                    ┌──────────────┐    
                    │ STATE_ERROR  │ (No wall detected)    
                    └──────────────┘
```

#### PI Controller for Wall-Following

Combines distance and angle control:

```
int16_t calculateSteering(const WallMeasurement& wall) {    
  // 1. Distance P-Controller    
  int16_t distanceError = wall.avgDistance - targetWallDistance;    
  int16_t distanceSteering = 0;    
      
  if (abs(distanceError) > 20) {  // 20mm dead zone    
    distanceSteering = -(int16_t)(distanceError * 0.15);    
  }    
      
  // 2. Angle PI-Controller    
  float angleSteering = 0.0;    
      
  if (abs(wall.angleToWall) > 2.0) {  // 2° dead zone    
    // P-component    
    float angleP = -wall.angleToWall * 2.0;    
        
    // I-component (accumulate over time)    
    float deltaTime = (millis() - lastUpdateTime) / 1000.0;    
    angleErrorIntegral += -wall.angleToWall * deltaTime;    
    angleErrorIntegral = constrain(angleErrorIntegral, -33.3, 33.3);    
        
    float angleI = angleErrorIntegral * 0.3;    
        
    angleSteering = angleP + angleI;    
  } else {    
    angleErrorIntegral *= 0.95;  // Decay in dead zone    
  }    
      
  // 3. Combine (40% angle, 60% distance)    
  float combined = 0.6 * distanceSteering + 0.4 * angleSteering;    
      
  return constrain((int16_t)combined, -45, 45);    
}
```

**Why PI instead of PID?**

- P: Immediate response to errors

- I: Handles curved walls automatically

- D: Not needed (increases noise sensitivity)

#### Dynamic Speed Control

```
int16_t calculateDynamicSpeed(const int16_t* distances) {    
  int16_t frontLeft = distances[7];    
  int16_t frontRight = distances[5];    
  int16_t minDistance = min(frontLeft, frontRight);    
      
  if (minDistance < 200)  return 0;      // STOP    
  if (minDistance < 400)  return speed * 0.25;    
  if (minDistance < 600)  return speed * 0.50;    
  if (minDistance < 800)  return speed * 0.75;    
  if (minDistance < 1000) return speed * 0.90;    
  return speed;  // Full speed    
}
```

### Core 1: Sensor Processing

Dedicated task runs independently on Core 1:

```
void sensorTask(void* parameter) {    
  while (true) {    
    // 1. Get list of active sensors (thread-safe)    
    uint8_t activeSensors[8];    
    uint8_t activeCount = getActiveSensors(activeSensors);    
        
    // 2. Perform 5 measurement passes    
    for (uint8_t pass = 0; pass < 5; pass++) {    
      for (uint8_t i = 0; i < activeCount; i++) {    
        uint8_t sensor = activeSensors[i];    
        uint16_t dist = measureDistance(sensor);    
        rawSamples[sensor][pass] = (dist == 0) ? -1 : dist;    
        delay(2);  // Prevent crosstalk    
      }    
      delay(60);  // Between passes    
    }    
        
    // 3. Apply median filter    
    int16_t filtered[8];    
    for (uint8_t s = 0; s < 8; s++) {    
      if (isSensorActive(s)) {    
        filtered[s] = calculateFilteredValue(rawSamples[s], 5);    
      } else {    
        filtered[s] = -2;  // Inactive marker    
      }    
    }    
        
    // 4. Apply calibration    
    applyCalibration(filtered);    
        
    // 5. Update global data (thread-safe)    
    updateSensorData(filtered);    
        
    delay(50);    
  }    
}
```

**Median Filter:**

1. Sort 5 measurements

2. Discard min and max

3. Average middle 3 values

4. Result: Immune to single outliers

**Cycle Time:**

- 4 active sensors: ~340ms

- 8 active sensors: ~700ms

### I2C Communication Protocol

MC2 acts as I2C master to MC1:

```
// High-level functions (I2C_Communication_EN.cpp)    
void driveForward(int16_t speed) {    
  setSpeed(-speed);  // Negative = forward in MC1    
}    
    
void setSteering(int16_t angle) {    
  sendI2CRegister(REG_STEERING, angle);    
}    
    
void sendI2CRegister(uint8_t reg, int16_t value) {    
  Wire.beginTransmission(0x10);  // MC1 address    
  Wire.write(reg);    
  Wire.write((value >> 8) & 0xFF);  // High byte    
  Wire.write(value & 0xFF);          // Low byte    
  Wire.endTransmission();    
}
```

**Register Map:**

| Register | Address | Type | Range | Function |
| - | - | - | - | - |
| SPEED | 0x01 | int16 | -800 to +800 | Steps/s (negative=forward) |
| STEERING | 0x02 | int16 | -90 to +90 | Steering angle (degrees) |
| COMMAND | 0x03 | uint8 | 0x01-0xFF | Special commands |


**Command Codes:**

| Code | Name | Function |
| - | - | - |
| 0x01 | CMD_STOP | Gradual stop (with ramp) |
| 0x02 | CMD_CENTER_STEERING | Reset steering to 0° |
| 0xFF | CMD_EMERGENCY_STOP | Immediate stop (no ramp) |


## Key Configuration Parameters

Located in `UserSettings_EN.h`:

```
// Wall following    
int16_t targetWallDistance = 200;     // mm from wall    
int16_t driveSpeed = 400;             // steps/s    
    
// Steering limits      
int16_t maxSteeringAngle = 45;        // degrees    
    
// Distance controller    
const float DISTANCE_P_GAIN = 0.15;   // Steering per mm error    
const int16_t DISTANCE_TOLERANCE = 20; // Dead zone (mm)    
    
// Angle controller (PI)    
const float ANGLE_P_GAIN = 2.0;       // P gain    
const float ANGLE_I_GAIN = 0.3;       // I gain    
const float ANGLE_I_LIMIT = 10.0;     // Max integral output    
const float MIN_ANGLE_CORRECTION = 2.0; // Dead zone (degrees)    
    
// Weighting    
const float ANGLE_WEIGHT = 0.4;       // 40% angle, 60% distance
```

## Performance Metrics

Measured on actual hardware:

### MC1 (LGT8F328P @ 16MHz)

```
Component               Time        Notes    
────────────────────────────────────────────    
ISR execution           ~30µs       4 motors checked    
I2C receive interrupt   ~50µs       3-byte write    
Main loop iteration     ~1ms        With delays    
Motor ramp update       ~100µs      Per motor    
Servo ramp update       ~80µs       Per servo
```

**CPU Usage:** ~10% (mostly idle, ISR + delays)

### MC2 (ESP32 @ 240MHz)

```
Task                    Time        Core    
────────────────────────────────────────────    
Sensor measurement      ~340ms      Core 1  (4 active)    
Display update          ~15ms       Core 0  (200ms rate-limited)    
PI controller calc      <1ms        Core 0    
I2C transmission        <1ms        Core 0    
Main loop iteration     ~10ms       Core 0
```

**Memory Usage:**

- Program: ~45% flash

- RAM: ~28% (sensor buffers + TFT)

### System Latency

```
Event                         Latency    
─────────────────────────────────────────    
Sensor → Navigation decision  ~350ms    
Decision → MC1 I2C command    <2ms    
MC1 command → Motor response  ~1ms (ISR)    
Total perception-to-action    ~350ms
```

## Debugging Tools

### MC1 Serial Debug

Add to MC1 for troubleshooting:

```
void setup() {    
  Serial.begin(115200);    
  // ...    
}    
    
void loop() {    
  static unsigned long lastDebug = 0;    
  if (millis() - lastDebug > 1000) {    
    Serial.print("Speed: "); Serial.print(regSpeed);    
    Serial.print(" Steering: "); Serial.println(regSteering);    
    lastDebug = millis();    
  }    
  // ...    
}
```

### MC2 Serial Debug

Already includes Serial output for sensor data.

### TFT Display

Real-time feedback:

- Sensor distances (or "OFF"/"ERR")

- Current system state

- Speed and wall distance

**Next Section:** [Build Instructions](#build-instructions)

