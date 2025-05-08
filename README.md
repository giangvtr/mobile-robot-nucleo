# Mobile Robot with Nucleo (Cortex-M4)

This project uses an **STM32 Nucleo board (Cortex-M4)** to build a basic mobile robot platform. The project is developed in **Keil µVision v5** and organized for embedded C development using CMSIS and STM32 HAL libraries.

---

## 📦 Project Overview

### Goals:
- Control **DC motors via PWM** for directional movement.
- Integrate an **ultrasonic distance sensor** (HC-SR04 or similar) to detect obstacles.

---

## Features Implemented

### Session 1: Motor Control
* Configured **PWM channels** on STM32 timers.
* Implemented basic robot **movement commands**:
  - Forward
  - Backward
  - Turn left / right
  - Stop

### Session 2: Ultrasonic Sensor Integration
* Configured GPIO pins for **TRIG** and **ECHO**.
* Measured distance using **input capture** or **timing logic**.
> ⚠️ Make sure to level-shift the ECHO signal if needed to protect GPIO pins.

---

## 🔌 Hardware Setup

| Component              | Description                          |
|------------------------|--------------------------------------|
| **Nucleo Board**       | STM32F4 series (Cortex-M4)           |
| **Motors**             | DC motors with H-bridge driver (e.g., L298N) |
| **Ultrasound Sensor**  | HC-SR04 (5V), interfaced via voltage divider |
| **Power Supply**       | 5V for logic, 9–12V for motors       |

---

## Development Environment
- **IDE**: Keil µVision 5
- **Language**: C (bare-metal)
- **Hardware debugger**: LogicPort

## Folder Structure
```plaintext
├── Core/               # Source files (main.c, pwm.c, ultrasonic.c, etc.)
├── DebugConfig/        # Keil debug files (ignored by Git)
├── Listings/           # Auto-generated build reports
├── Objects/            # Compiled object files
├── RTE/                # CMSIS and device configuration
├── .gitignore          # Git settings
├── README.md           # You're here!
```
---

## How to Run

1. Clone this repository:
   ```bash
   # With SSH
   git clone git@github.com:giangvtr/mobile-robot-nucleo.git

   # With HTTPS:
   git clone https://github.com/giangvtr/mobile-robot-nucleo.git
   # Then sign in with your user name and password
   ```
2. Open Keil µVision software
- Go to File > Open Project
- Choose `uvprojx` file type then choose `TP_SMP.uvprojx`

3. Setup debugging environment (directly on hardware)
4. Power the card
5. Build, debug and run the project

---


