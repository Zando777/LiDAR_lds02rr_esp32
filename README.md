# LiDAR LDS02RR ESP32 Interface

This project interfaces with the Xiaomi LDS02RR LiDAR sensor using an ESP32 microcontroller. It reads scan data from the LiDAR and outputs it via serial monitor.

## Features

- Reads 360-degree scan data from LDS02RR LiDAR
- Outputs scan points with angle and distance measurements
- Configurable scan frequency (set to 5 Hz)
- Debug options for missed angles and packet inspection
- PWM motor control for LiDAR

## Hardware Setup

- ESP32 (NodeMCU-32S)
- LDS02RR LiDAR connected via UART:
  - LiDAR TX → ESP32 GPIO 21 (RX)
  - LiDAR PWM motor pin → ESP32 GPIO 15

## Software Dependencies

- PlatformIO
- Arduino framework for ESP32
- LDS library (kaiaai/LDS@^0.6.3)

## Configuration

- Serial monitor baud rate: 250000
- LiDAR serial RX buffer: 8192 bytes
- PWM frequency: 10kHz, 11-bit resolution

## Output Format

Scan data is output in CSV format:
```
SCAN_START
angle1,distance1
angle2,distance2
...
SCAN_END
```

Individual points are also printed every 50th point.

## Build and Upload

Use PlatformIO to build and upload the project to ESP32.