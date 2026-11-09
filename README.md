# ⚡ Focal CNC — Real-Time Laser Cutter & Engraver Firmware

> **Professional High-Performance Real-Time Firmware for CNC Laser Cutters & Engravers on ESP32-S3 Dual-Core with Apple-Inspired Touch GUI.**

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC_BY--NC--SA_4.0-lightgrey.svg)](LICENSE)
[![Target: ESP32-S3](https://img.shields.io/badge/Target-ESP32--S3_N16R8-red.svg)](https://www.espressif.com/)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4-blue.svg)](https://github.com/espressif/esp-idf)
[![GUI: LVGL](https://img.shields.io/badge/GUI-LVGL_v8.3-orange.svg)](https://lvgl.io/)
[![Kinematics: Dual-Y](https://img.shields.io/badge/Kinematics-Dual--Y_T8x8-green.svg)]()
[![Laser: 20W PWM](https://img.shields.io/badge/Laser-Tree_Laser_20W_PWM-blueviolet.svg)]()
[![Author: Jhoel Ospino](https://img.shields.io/badge/Author-Jhoel_Ospino-purple.svg)](https://github.com/Jhoel-ospino)

---

## 📖 Overview

**Focal CNC** is a production-grade, real-time motion control and human-machine interface firmware engineered specifically for high-precision optical laser cutters and engravers. Built from the ground up for the **ESP32-S3 (Xtensa Dual-Core 240 MHz, 16MB Flash, 8MB Octal PSRAM)**, it combines deterministic motion execution with a fluent, Apple-inspired touch user interface:

- **FreeRTOS Dual-Core Architecture**: 
  - **Core 0**: LVGL v8 graphics rendering (60 FPS), Wi-Fi REST API, Web Server, and USB Serial streaming.
  - **Core 1**: Real-time deterministic Bresenham motion engine with trapezoidal velocity profiling and lockstep dual-axis synchronization.
- **Synchronized Dual-Y Kinematics**: Drives two independent NEMA 17 motors on parallel T8 metric 4-start lead screws in exact lockstep, preventing gantry racking without mechanical linkage.
- **Tree Laser 20W Optical Module**: 5 kHz 10-bit PWM control with dynamic feedrate laser power scaling (GRBL Laser Mode M4) to eliminate burned corners.
- **Apple-Inspired Focal Design System**: Clean pastel visual language, high border radii, monolithic vector icons, and dynamic Liquid Wave live cut progress monitor.
- **Autonomous MicroSD Execution**: Compiles and runs complete `.gcode` programs locally from FATFS MicroSD card to prevent job failure due to PC disconnections.

---

## 🔌 Hardware Pinout Configuration (ESP32-S3 N16R8)

All pins are strictly decoupled from the internal 8MB Octal PSRAM/Flash bus (GPIO 33–37 reserved):

| Peripheral / Module | Pin / Signal | ESP32-S3 GPIO | Logic Level & Characteristics |
|---|---|---|---|
| **Motors Enable** | Common `ENABLE` | **GPIO 3** | Active LOW (TMC2209 StepStick enable) |
| **X-Axis Motor** | `STEP_X` | **GPIO 1** | Rising edge pulse (High-speed output) |
| **X-Axis Motor** | `DIR_X` | **GPIO 2** | Direction level (CW / CCW) |
| **Y1-Axis Motor (Left)** | `STEP_Y1` | **GPIO 4** | Synchronized Dual-Y pulse |
| **Y1-Axis Motor (Left)** | `DIR_Y1` | **GPIO 5** | Direction level |
| **Y2-Axis Motor (Right)**| `STEP_Y2` | **GPIO 6** | Synchronized Dual-Y pulse (Lockstep) |
| **Y2-Axis Motor (Right)**| `DIR_Y2` | **GPIO 7** | Direction level |
| **Laser Diode 20W** | `LASER_PWM` | **GPIO 18** | LEDC Timer 0, 5 kHz, 10-bit (0–1000 duty) |
| **Passive Buzzer** | `BUZZER` | **GPIO 38** | LEDC PWM (Job completion / Alarm alerts) |
| **TFT Display (ILI9488)**| `MOSI` / `CLK` / `MISO` | **GPIO 11 / 12 / 13** | SPI2 Bus (Host 1), 40 MHz DMA |
| **TFT Display (ILI9488)**| `CS` / `DC` / `RST` | **GPIO 10 / 9 / 14** | Display control lines |
| **Touchscreen (XPT2046)**| `T_CS` / `T_IRQ` | **GPIO 21 / GPIO 39** | Dedicated CS and IRQ |
| **MicroSD Card (Slot)** | `SD_CS` | **GPIO 47** | SPI2 shared bus with dedicated CS |
| **Limit Switches** | `LIMIT_X` / `LIMIT_Y` | **GPIO 16 / GPIO 17** | Internal Pull-Up, Active LOW (Future homing) |

---

## 📐 Kinematics & Mechanics

The machine mechanics are based on metric **T8 lead screws with 4-start threads**:
- **Pitch**: 2 mm
- **Lead (Avance)**: 8 mm per complete revolution.
- **Motor Step Angle**: 1.8° (200 full steps per revolution).
- **Driver Microstepping**: 16 microsteps (TMC2209 Standalone).

$$	ext{Steps per mm} = rac{200 	imes 16}{8	ext{ mm}} = \mathbf{400.0	ext{ steps/mm}}$$

- **Default Bed Dimensions**: 200 mm (X) × 160 mm (Y) — configurable up to 400×400 mm in Settings.
- **Rapid Travel (Feedrate)**: Up to 4000 mm/min.
- **Acceleration**: 800 mm/s².

---

## 🚀 Building and Flashing

### Requirements
- [ESP-IDF v5.4+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/)
- Python 3.10+
- USB-C cable connected to native USB port (`USB-Serial/JTAG`).

### 1. Clone the Repository
```bash
git clone https://github.com/Jhoel-ospino/FOCAL_CNC_LASER.git
cd FOCAL_CNC_LASER
```

### 2. Configure & Build
```bash
# Load ESP-IDF environment
source $HOME/esp/esp-idf/export.sh

# Set target to ESP32-S3
idf.py set-target esp32s3

# Compile firmware
idf.py build
```

### 3. Flash to Target
```bash
# Replace /dev/cu.usbmodem11301 with your device serial port
idf.py -p /dev/cu.usbmodem11301 flash monitor
```

---

## 🖥️ Companion Ecosystem

- **[Focal Studio Designer](https://github.com/Jhoel-ospino/focal-studio)**: Visual UI designer and C code synthesizer for the touchscreen.
- **[Focal Sim](https://github.com/Jhoel-ospino/focal-sim)**: Desktop SDL2 simulator with 1:1 hardware mocks and 29 automated regression tests.

---

## ⚖️ License & Intellectual Property

**Copyright (c) 2026 Jhoel Ospino. All rights reserved.**

This project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0)**.

- **Non-Commercial Only**: Commercial exploitation, distribution, sale, or integration into commercial hardware products without prior written authorization from the copyright owner (Jhoel Ospino) is strictly prohibited.
- **Community & Education**: Open for personal, research, academic, and non-commercial community maker use.
- **Attribution**: Proper attribution to **Jhoel Ospino** must be clearly preserved in all forks and derivative projects.
