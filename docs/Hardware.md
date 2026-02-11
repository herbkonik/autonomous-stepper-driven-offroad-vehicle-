# Hardware Setup

## System Architecture Overview

The robot is built around a two-microcontroller architecture that separates motor control from navigation logic:

```
┌─────────────────────────────────────────────────┐  
│              MC2 (ESP32-WROOM-32U)              │  
│            Navigation & Sensor Brain            │  
│  ┌──────────────┐         ┌─────────────────┐   │  
│  │   Core 0     │         │     Core 1      │   │  
│  │ - Display    │         │ - Sensors (8x)  │   │  
│  │ - I2C Master │         │ - Filtering     │   │  
│  │ - Navigation │         │ - Calibration   │   │  
│  └──────┬───────┘         └────────┬────────┘   │  
│         │                          │            │  
│         └──────────┬───────────────┘            │  
│                    │ Mutex                      │  
└────────────────────┼────────────────────────────┘  
                     │ I2C (3.3V)  
┌────────────────────┼────────────────────────────┐  
│                                                 │  
│          MC1 (LGT8F328P LQFP48)                 │  
│              Motor Control Unit                 │  
│  ┌──────────────────────────────────────────┐   │  
│  │ - FOC Stepper Drivers (4x)               │   │  
│  │ - Steering Servo Control (4x)            │   │  
│  │ - I2C Slave Interface                    │   │  
│  │ - Timer-based Step Generation            │   │  
│  └──────────────────────────────────────────┘   │  
└─────────────────────────────────────────────────┘  
         │                    │  
         ▼                    ▼  
   [4x NEMA17 FOC\]    [4x MG946R Servos\]
```

## Why This Architecture?

### MC1 (Motor Control) - LGT8F328P

**Selected for:**

- **Voltage Compatibility:** Designed for 5V but fully operational at 3.3V without limitations

- **Direct I2C Connection:** No level shifters needed with ESP32 (both at 3.3V)

- **ATmega328 Compatible:** Familiar Arduino ecosystem, mature libraries

- **Hardware PWM:** Multiple timers for stepper control and servo signals

- **Cost Effective:** ~$2-3 per chip

- **Dedicated Task:** Handles real-time motor control without OS overhead

### MC2 (Navigation) - ESP32-WROOM-32U

**Selected for:**

- **Dual-Core:** Separate sensor processing from navigation logic

- **FreeRTOS:** Built-in multitasking support

- **WiFi/Bluetooth:** Future telemetry and remote debugging

- **Fast Processing:** 240MHz for PI controller calculations

- **Large Memory:** 520KB RAM for sensor buffering

- **Rich Peripheral Set:** Multiple I2C, SPI, UART interfaces

## Bill of Materials

### MC1 - Motor Control Unit

| Component | Specification | Quantity | Notes |
| - | - | - | - |
| **Microcontroller** |  |  |  |
| LGT8F328P LQFP48 MiniEVB | 16MHz, 3.3V operation | 1 | 5V-rated chip running at 3.3V |
| **Motor Drivers** |  |  |  |
| MKS Servo42C | FOC Closed-Loop | 4 | Critical for performance |
| NEMA17 Steppers | 1.5A+ rated, 200 steps/rev | 4 | Higher current = more torque |
| **Steering** |  |  |  |
| MG946R Servos | Metal gear, 270 deg | 4 | One per wheel |


**Key Feature:** The LGT8F328P's 3.3V compatibility eliminates the need for I2C level shifters between MC1 and MC2, simplifying wiring and improving reliability.

### MC2 - Navigation Brain

| Component | Specification | Quantity | Notes |
| - | - | - | - |
| **Microcontroller** |  |  |  |
| ESP32-WROOM-32U | Dual-core, 3.3V | 1 | Any ESP32 variant works |
| **Sensors** |  |  |  |
| HC-SR04 Ultrasonic (CS100A) | 3.3V compatible | 8 | Works at 3.3V with reduced range |
| **Display** |  |  |  |
| TFT Display | ST7789VW 240x320, 2.0" IPS | 1 | SPI interface |
| **I2C** |  |  |  |
| Pull-up Resistors | 4.7k Ohm | 2 | SDA and SCL to 3.3V |
| **Decoupling** |  |  |  |
| Capacitors | 100nF ceramic | 8 | One near each HC-SR04 |


### Mechanical & Power

| Component | Specification | Quantity | Notes |
| - | - | - | - |
| **Chassis** | Custom, supports 10kg+ | 1 | Offroad-capable |
| **Wheels** | Offroad tread, ~100mm dia | 4 | Rubber or foam |
| **Wire** | Silicone 18-22 AWG | - | For motor power |
| **Battery** | 12-28V, 5000mAh+ | 1 | LiPo, Li-ion, or lead-acid |
| **Buck Converter** | 28V to 12V, 10A | 1 | For FOC motor drivers |
| **Buck Converter** | 28V to 6.5V, 5A | 1 | For servos |
| **Buck Converter** | 28V to 3.3V, 3A | 1 | For electronics (MC1, MC2, sensors) |
| **Mounting Hardware** | M3 screws/standoffs | - | As needed |


## MC1 Pin Assignments (LGT8F328P)

### Stepper Motor Connections

```
// Motor IDs and Pin Assignments  
MOTOR_FRONT_LEFT:   Step=3,  Dir=4,  Enable=31  
MOTOR_FRONT_RIGHT:  Step=34, Dir=2,  Enable=33  
MOTOR_REAR_LEFT:    Step=9,  Dir=10, Enable=11  
MOTOR_REAR_RIGHT:   Step=21, Dir=25, Enable=20
```

**Connection to MKS Servo42C:**

- **STEP:** Pulse train for motion (3.3V TTL compatible)

- **DIR:** Direction control (HIGH/LOW)

- **ENABLE:** Active LOW to enable driver

### Servo Connections

```
// Servo IDs and PWM Pins  
SERVO_FRONT_LEFT:   Pin=5   (PWM via ServoTimer2)  
SERVO_FRONT_RIGHT:  Pin=24  (PWM via ServoTimer2)  
SERVO_REAR_LEFT:    Pin=12  (PWM via ServoTimer2)  
SERVO_REAR_RIGHT:   Pin=26  (PWM via ServoTimer2)
```

**PWM Signal:** 1500us center, 600-2400us range (270 deg servos)

### I2C Slave Interface

```
// I2C Pins (3.3V logic)  
SDA: Standard I2C pin (varies by board)  
SCL: Standard I2C pin (varies by board)  
Address: 0x10 (MC1 slave address)
```

**Important:** LGT8F328P operates at 3.3V in this application - direct connection to ESP32 I2C without level shifting!

## MC2 Pin Assignments (ESP32)

### HC-SR04 Ultrasonic Sensors

**Trigger Pins (Output):**

```
Sensor 0 (L-Side-Rear):  GPIO 32  
Sensor 1 (L-Rear):       GPIO 25  
Sensor 2 (R-Rear):       GPIO 27  
Sensor 3 (R-Side-Rear):  GPIO 12  
Sensor 4 (R-Side-Front): GPIO 15  
Sensor 5 (R-Front):      GPIO 4  
Sensor 6 (L-Side-Front): GPIO 13  
Sensor 7 (L-Front):      GPIO 5
```

**Echo Pins (Input):**

```
Sensor 0: GPIO 35 (INPUT_ONLY)  
Sensor 1: GPIO 33  
Sensor 2: GPIO 26  
Sensor 3: GPIO 14  
Sensor 4: GPIO 2  
Sensor 5: GPIO 39 (VN - INPUT_ONLY)  
Sensor 6: GPIO 36 (VP - INPUT_ONLY)  
Sensor 7: GPIO 34 (INPUT_ONLY)
```

**Note:** HC-SR04 CS100A sensors operate at 3.3V with slightly reduced range compared to 5V operation. No voltage level conversion needed!

### I2C Master Interface

```
SDA: GPIO 21  
SCL: GPIO 22  
Clock: 100kHz  
Pull-ups: 4.7k Ohm to 3.3V (both lines)
```

### TFT Display

Uses hardware SPI:

```
MOSI: GPIO 23  
SCLK: GPIO 18  
CS:   GPIO 15 (configurable)  
DC:   GPIO 2  (configurable)  
RST:  GPIO 4  (configurable)
```

*Configure in TFT_eSPI User_Setup.h*

## Sensor Placement Strategy

Position sensors in a ring around the vehicle:

```
        [7-Front-L]  \[5-Front-R]  
              \         /  
               \       /  
                \ CAR /  
    [6-Side-L]   |   |   \[4-Side-R]  
                 |   |  
                 | ▼ |  (Direction)  
    [0-Side-L]   |   |   \[3-Side-R]  
                /     \  
               /       \  
       [1-Rear-L]   [2-Rear-R]
```

**Critical Distances:**

- **Side sensors (0\<-\>6 and 3\<-\>4):** 600mm apart (SIDE_SENSOR_DISTANCE constant)

- **Height:** 150-200mm above ground for best range

- **Angle:** Slightly downward (5-10 deg) for ground obstacle detection

**Mounting Tips:**

1. Use foam tape to dampen vibrations

2. Keep sensors clear of wheels/moving parts

3. Add small hoods to reduce direct sunlight interference

4. Ensure left/right symmetry for accurate angle calculations

## Power Distribution

```
                Battery (12-28V, 5000mAh+)  
                          |  
         ┌────────────────┼─────────────────┐  
         |                |                 |  
    [Buck 28→12V]    [Buck 28→6.5V]   [Buck 28→3.3V]  
       10A output       5A output        3A output  
         |                |                 |  
         |                |                 |  
    [FOC Drivers]     [Servos 4x]    [MC1 + MC2 + Sensors]  
      (4x @ 12V)       (6.5V)              (3.3V)  
         |                |                 |  
         └────────────────┴─────────────────┘  
                          |  
                   [Vehicle Ground]
```

**Power Budget:**

- FOC Drivers (4x, running): 12V x 1.5A x 4 = 72W

- Servos (4x, peak): 6.5V x 1A x 4 = 26W

- ESP32: 3.3V x 500mA = 1.65W

- LGT8F328P: 3.3V x 100mA = 0.33W

- Sensors (8x): 3.3V x 15mA x 8 = 0.4W

- Display: 3.3V x 150mA = 0.5W

- **Total peak:** ~100W

**Battery Runtime Estimate:**

- 5000mAh @ 24V = 120Wh capacity

- Average draw ~60W = ~2 hours runtime

- Peak draw ~100W = ~1.2 hours runtime

**Voltage Range:** 12-28V input allows flexibility:

- 3S LiPo: 11.1V nominal (9.9-12.6V)

- 4S LiPo: 14.8V nominal (13.2-16.8V)

- 5S LiPo: 18.5V nominal (16.5-21V)

- 6S LiPo: 22.2V nominal (19.8-25.2V)

- Lead-acid: 12V/24V systems

## Assembly Notes

### 1. MC1 (LGT8F328P) Setup

**Critical:** Run LGT8F328P at 3.3V for I2C compatibility with ESP32!

**Programming:**

- Use FTDI or CP2102 USB-Serial adapter (set to 3.3V)

- Arduino IDE: Select "LGT8F328P" board

- Upload MC1 firmware via Serial

**Wiring:**

1. Connect all stepper STEP/DIR/ENABLE pins

2. Connect all servo PWM pins

3. Wire I2C directly to ESP32 (no level shifter needed)

4. Add pull-ups (4.7k Ohm) on both SDA and SCL

### 2. MC2 (ESP32) Setup

**Sensor Connections:** All HC-SR04 CS100A sensors connect directly to ESP32:

- VCC to 3.3V rail

- GND to Common ground

- TRIG to ESP32 GPIO (output pins)

- ECHO to ESP32 GPIO (input pins)

**No voltage dividers needed!** The CS100A variant operates at 3.3V natively.

**Decoupling:**

- Place 100nF capacitor near each HC-SR04 VCC pin

- Large capacitor (100-470uF) near ESP32 VIN

### 3. I2C Bus (MC1 \<-\> MC2)

**Shared 3.3V I2C Bus:**

```
ESP32 GPIO21 (SDA) ─────┬───── LGT SDA  
                    4.7k Ohm  
                        |  
                       3.3V  
  
ESP32 GPIO22 (SCL) ─────┬───── LGT SCL  
                    4.7k Ohm  
                        |  
                       3.3V
```

**No level shifters required** - both MCUs at 3.3V logic!

### 4. Sensor Calibration

Measure each sensor against a flat wall:

```
// In Config_EN.h:  
const int16_t SENSOR_OFFSETS\[8] = {  
  -20,  // Sensor 0: reads 20mm short  
  +15,  // Sensor 1: reads 15mm long  
  // ... measure and adjust for all 8  
};
```

## Testing Checklist

**MC1 Tests:**

- [ ] All motors respond to step pulses

- [ ] Direction changes correctly

- [ ] All servos center at 1500us

- [ ] I2C acknowledges from MC2

**MC2 Tests:**

- [ ] All 8 sensors read valid distances

- [ ] Display shows sensor values

- [ ] I2C commands reach MC1

- [ ] Both cores running (check via Serial)

**Power System:**

- [ ] Buck converters output correct voltages

- [ ] All voltage rails stable under load

- [ ] No voltage drops during motor startup

- [ ] Battery monitoring functional (if implemented)

**System Integration:**

- [ ] MC2 can set motor speed via I2C

- [ ] MC2 can control steering via I2C

- [ ] Emergency stop works

- [ ] No loose wires near moving parts

## Optional Enhancements

- **IMU (MPU6050):** Better orientation tracking

- **GPS Module:** Outdoor waypoint navigation

- **WiFi Telemetry:** Stream sensor data remotely

- **Voltage Monitor:** Low-battery warning on ESP32 ADC

- **Bumper Switches:** Mechanical backup sensors

- **Current Sensors:** Monitor motor load (ACS712 or INA219)

- **Status LEDs:** Visual feedback without display


**Next Section:** [Software Architecture](#software-architecture)

