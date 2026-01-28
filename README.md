# Avionics_FC (Arduino Nano) 🚀  
**Arduino-based Flight Computer firmware **

An early-stage flight computer firmware built for **Arduino-class microcontrollers**, using a **single-loop architecture** with modular subsystems for sensing, logging, command handling, and flight-mode logic.

> ⚠️ **Prototype / v1 firmware**  
> This repository is a functional baseline but is **NOT flight-certified** and has not been fully validated under real flight conditions.  
> Use for bench testing, subsystem bring-up, and learning purposes.

---

## Features

- Single-loop design  
- Modular subsystems for:
  - IMU sensing  
  - Barometric pressure / altitude estimation  
  - SD card data logging  
  - HC-05 Bluetooth command interface  
- **1D Kalman Filter** for altitude estimation  
- Apogee detection using fused IMU + barometer data  
- State-machine style flight logic (modes)

---

## Architecture Overview

The firmware is organized around a **main loop** that periodically calls each subsystem in a deterministic order.

### Main Loop Responsibilities

- **Sensor Update**
  - Read IMU
  - Read barometer
  - Compute raw altitude

- **Estimation**
  - Run 1D Kalman Filter
  - Produce filtered altitude and vertical state

- **Flight Logic**
  - Evaluate mode transitions
  - Detect apogee

- **Logging**
  - Write sensor + state data to SD card

- **Command Handling**
  - Receive commands via HC-05 Bluetooth
  - Update parameters / modes

This design favors **simplicity and determinism** over concurrency.

---

## Hardware / Intended Setup

Typical Arduino-based avionics stack:

- Arduino Uno / Nano / Mega (or compatible)
- IMU sensor - MPU6050 (I2C/SPI)
- Barometer / pressure sensor BME280
- microSD card module (SPI)
- HC-05 Bluetooth module (UART)

> Pin definitions and driver assumptions must match your specific board and wiring.

---

## Build / Flash

### Tooling

- Arduino IDE  
- Appropriate board package installed  

### Upload

1. Select correct Arduino board  
2. Select COM port  
3. Compile & Upload  

---

## Repository Structure

- `FC2.0/`  
  Main firmware project directory (Arduino-style layout)

---

## Known Issues / Limitations

### 1) No real multitasking
- All subsystems run sequentially
- Slow SD writes or blocking I/O can affect timing

### 2) Timing jitter
- Loop execution time varies with sensor and SD latency

### 3) Limited RAM
- Kalman filter + buffers + logging must fit in small SRAM

### 4) Serial blocking risks
- Bluetooth and USB serial operations may block execution

### 5) Not flight-certified logic
- Mode logic requires:
  - extensive ground testing  
  - calibration  
  - watchdog integration  
  - fault handling  

Treat as a prototype.

---

## Disclaimer

This code is published solely for LEARNING PURPOSES.  
If you intend to use it in real flight hardware, you **must perform full validation and safety testing**.

---

## Note from Creator

This was purely for learning and getting hands-on with embedded systems.  
I am freezing this project here and **DO NOT** plan to actively develop it unless a need arises.

---

## Author

Anand  
Repo: `anand1946s/Avionics_FC`
