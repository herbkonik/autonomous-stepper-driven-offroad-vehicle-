# Autonomous Stepper-Driven Off-Road Vehicle

<img width="800" height="491" alt="RDS-Carrier_02" src="https://github.com/user-attachments/assets/cd4c19a0-782f-4926-b813-53f46fde71f7" />


Experimental autonomous off-road vehicle using stepper motors with advanced control
methods. The project investigates whether stepper motors can be a viable alternative
to DC motors for low-speed, high-torque outdoor vehicles.

## Project Story & Build Log

The project is still work in progress, a detailed project story, background, and build documentation i will add to Hackster.io. 

## Motivation

Conventional wisdom states that *“stepper motors don’t work for vehicles”* due to step loss,
poor efficiency, and limited torque under load. However, off-road autonomous vehicles
operating on agricultural terrain require high torque at very low speeds — exactly where
small DC motors struggle.

This project explores whether advanced control strategies, such as field-oriented control
(FOC), combined with a suitable mechanical design, can overcome these limitations.

## System Overview

- Autonomous off-road vehicle for agricultural terrain
- Rocker-differencing suspension
- Stepper motors integrated into wheel hubs
- Four-wheel steering (MGR946R)
- Dual-MCU architecture for real-time motor control and high-level autonomy
- Developed with VS Code and PlatformIO

## Architecture

### Dual-MCU Concept

- **MCU Motion**  
  Low-level real-time motor control, stepper drive, and torque generation.

- **MCU Control**  
  High-level vehicle logic, planning, state management, and sensor integration.

Communication between the MCUs is handled via a dedicated interface (details in code).

## Repository Structure

src/

├── mcu_motion/ # LGT8F328P: Low-level motor control, steering

├── mcu_control/ # ESP32 Wroom: High-level control, planning & autonomy



## License
This project is licensed under the GNU GPLv3.

