# Avionics_FC 🚀  
**ESP32-based Flight Computer firmware (FreeRTOS)**

An early-stage flight computer firmware built for **ESP32 microcontrollers**, designed around a **FreeRTOS task architecture** with separate modules for sensing, telemetry, logging, and mode/state management.

> ⚠️ **Prototype / v1 firmware**
> This repository is a functional baseline but is **NOT flight-certified** and has not been fully validated under real flight conditions.  
> Use for bench testing, subsystem bring-up, and architecture reference.

---

## Features

- **FreeRTOS-based multi-tasking design**
- Modular drivers / subsystems for:
  - IMU sensing
  - Pressure / altitude estimation
  - GPS (recovery mode)
  - LoRa telemetry
  - SD logging
- State-machine style flight logic (modes)

---

## Architecture Overview

The firmware follows a common flight-computer separation:

### Tasks (conceptual)
- **Sensor Task**
  - Reads IMU + pressure sensor at fixed rate
  - Computes derived values (e.g., altitude)
  - Pushes `SensorData` into queues

- **Logger Task**
  - Pops data from queue
  - Writes to SD card / log function

- **Telemetry Task**
  - Pops latest available sensor packet
  - Sends via LoRa
  - Optionally injects GPS fields during recovery mode

- **Command Task** *(optional / early)*
  - Reads command input (ground station)
  - Updates parameters / mode if supported

- **Mode Manager Task**
  - Central state machine / mode control

### Communication
Inter-task data flow uses **FreeRTOS queues**:
- `loggerQueue`
- `telemetryQueue`

This avoids shared-state spaghetti and makes timing behavior more predictable.

---

## Hardware / Intended Setup

This firmware targets an ESP32-based avionics stack typically containing:

- ESP32 / ESP32-S3
- IMU sensor (I2C/SPI)
- Barometer / pressure sensor
- LoRa transceiver module
- GPS module (UART)
- microSD card module (SPI)

> Note: Wiring, sensor model, and LoRa module details depend on your actual build.  
> The code structure is modular, but pin definitions and driver assumptions must match your board.

---

## Build / Flash

### Tooling
- Arduino IDE or PlatformIO (Arduino framework)
- ESP32 board support installed

### Upload
1. Select correct ESP32 board
2. Select correct COM port
3. Compile & Upload

---

## Repository Structure

- `flightcomputer/`  
  Main firmware project directory (Arduino-style layout)

---

## Known Issues / Limitations 

This is a v1 baseline. Some important limitations:

### 1) Runtime stability not fully validated
- Under FreeRTOS, stability depends heavily on:
  - task stack sizes
  - heap usage
  - peripheral driver behavior under concurrency
  - 
### 2) Potential stack pressure in tasks
- Tasks doing sensor reads + math + logging/telemetry can exceed stack


### 3) Possible heap fragmentation risks
- If any command/GPS parsing uses Arduino `String`, heap fragmentation can lead to instability over time


### 4) Peripheral access concurrency
- Some Arduino drivers (Serial/I2C/SPI) are not inherently thread-safe
- Concurrent use from multiple tasks can cause undefined behavior

### 5) Data drops under queue overflow
- Queue sends can fail if consumers lag (telemetry/logging blocking)
- Current design prioritizes *real-time progress* over guaranteed delivery


### 6) Not flight-certified logic
- Mode transition logic needs:
  - real flight testing
  - fault handling
  - sensor calibration
  - watchdog strategy
  - redundant safety behavior
- Treat as a prototype :)

---


## Disclaimer

This code is published for educational and development purposes.  
If you intend to use it in real flight hardware, you **must perform full validation and safety testing**.

---

## Note from Creator
This was purely for learning and gettings hands on with embedded system programming. I am freezing this project here and **DO NOT** plan to work on this project unless a need arises


---

## Author

Anand  
Repo: `anand1946s/Avionics_FC`
