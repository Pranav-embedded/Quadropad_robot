# ESP32-CAM Servo Robot

A WiFi-controlled walking robot with live video streaming, built around an ESP32-CAM and Arduino Nano driving 12 servo motors — powered entirely from a 3S Li-ion battery pack.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
  - [Power System](#power-system)
  - [Control Stack](#control-stack)
  - [Motion System](#motion-system)
- [System Architecture](#system-architecture)
- [Power Rail Summary](#power-rail-summary)
- [Communication Protocol](#communication-protocol)
- [Software](#software)
- [Getting Started](#getting-started)
- [Known Issues & Fixes](#known-issues--fixes)
- [Project Status](#project-status)

---

## Overview

This robot streams live video from an onboard camera to a laptop dashboard while accepting motion commands over WiFi. The ESP32-CAM handles all wireless communication and delegates servo control to an Arduino Nano via serial. Twelve servo motors provide the degrees of freedom needed for walking gaits.

---

## Hardware

### Power System

| Component | Role | Input | Output |
|-----------|------|-------|--------|
| 3S 18650 Li-ion battery pack | Main energy source | — | ~12 V nominal |
| 3S 40 A BMS | Cell protection, overcurrent cutoff | 12 V | 12 V (protected) |
| XL4016 buck converter | Step-down for logic + servos | 12 V | 7 V |
| MP1548 buck converter | Step-down for ESP32-CAM | 7 V | 5 V |

### Control Stack

| Component | Description |
|-----------|-------------|
| ESP32-CAM | Main wireless controller — handles WiFi, video streaming, and command routing |
| Arduino Nano | Motion controller — receives serial commands and drives all 12 servos |
| Arduino Nano expansion shield | Breakout board providing servo headers and clean power distribution |

### Motion System

- **12 × servo motors** — connected to the Arduino Nano expansion shield
- Powered from the 7 V rail (XL4016 output) via the Arduino expansion shield
- Controlled via predefined motion scripts (sequences of joint angles) stored on the Nano

---

## System Architecture

```
START
  │
  ▼
Initialization
(power-on, hardware setup)
  │
  ▼
┌─────────────────────────────────────┐
│   Command & Video Streaming          │  ◄──────────────────┐
│   WiFi — bidirectional comms         │                     │
└──────────────────┬──────────────────┘                     │
                   │                                         │
                   ▼                                         │
      ESP32-CAM processing                                   │
      (parses and routes commands)                           │
                   │                                         │
                   ▼                                         │
      Command → Arduino Nano                                 │
      (Serial / I²C link)                                    │
                   │                                         │
                   ▼                                         │
      Arduino Nano execution                                 │
      (decodes motion command)                               │
                   │                                         │
                   ▼                                         │
      Predefined motion scripts                              │
      (sequence of joint angles)                             │
                   │                                         │
                   ▼                                         │
          Servo movement ──────────────────────────────────►─┘
                                     (continuous loop)
```

---

## Power Rail Summary

```
3S Li-ion (~12 V)
      │
      └─► BMS (3S, 40 A)
               │
               └─► XL4016 → 7 V rail
                        │
                        ├─► Arduino Nano expansion shield
                        │         └─► 12 × servo motors
                        │
                        └─► MP1548 → 5 V rail
                                  └─► ESP32-CAM module
```

> **Note:** Capacitors on the servo power rail are recommended to suppress voltage spikes during simultaneous servo movement, preventing brownout resets on the Arduino Nano (I used 1000 uF capacitor on servo power rail).

---

## Communication Protocol

### Laptop → ESP32-CAM (commands)

Commands are sent from the laptop dashboard to the ESP32-CAM over WiFi. The ESP32-CAM parses each command string and forwards the appropriate instruction to the Arduino Nano over UART serial (or I²C).

### ESP32-CAM → Laptop (video)

The ESP32-CAM streams MJPEG video directly to the laptop dashboard over WiFi. Stream parameters (JPEG quality, frame rate cap) are configurable in firmware to manage thermal load.

### ESP32-CAM → Arduino Nano (motion)

Motion commands are relayed from the ESP32-CAM to the Arduino Nano via the serial link on the expansion shields. The Nano decodes the command string and maps it to a predefined motion script.

---

## Software

### ESP32-CAM Firmware

- WiFi connection and HTTP server for video stream endpoint
- WebSocket or HTTP endpoint for receiving commands from dashboard
- Stream loop with frame rate cap to prevent thermal overload
- UART serial output to forward motion commands to Arduino Nano

### Arduino Nano Firmware

- UART serial listener for incoming command strings
- Command parser mapping strings to motion script IDs
- Motion script executor — steps through joint angle arrays with configurable delays
- Servo library driving all 12 channels via the expansion shield headers

### Laptop Dashboard

- Browser-based UI displaying live video stream
- Directional controls sending commands to ESP32-CAM over WiFi
- Bidirectional communication for status feedback

---

## Getting Started

### Requirements

- Arduino IDE (with ESP32 board package installed)
- Libraries: `esp_camera`, `WebServer`, `WiFi`, `HardwareSerial`, `Servo`
- Python or any HTTP server (for dashboard, if not served from ESP32)

### Flashing

1. Flash the Arduino Nano firmware first — ensure all 12 servos are centred before mounting.
2. Flash the ESP32-CAM firmware using an FTDI programmer or the expansion shield's USB interface.
3. Set your WiFi SSID and password in the ESP32-CAM firmware before flashing.
4. Power the system from the battery pack (or bench supply at 12 V during development).
5. Open the dashboard and connect to the ESP32-CAM's IP address.

### First Boot Checklist

- [ ] BMS indicator shows healthy cell balance
- [ ] XL4016 output verified at 7 V (adjust trim pot if needed)
- [ ] MP1548 output verified at 5 V (adjust trim pot if needed)
- [ ] All 12 servos respond to a centre-position command
- [ ] ESP32-CAM connects to WiFi and video stream is visible on dashboard
- [ ] Motion commands from dashboard produce correct gait

---

## Known Issues & Fixes

| Issue | Cause | Fix |
|-------|-------|-----|
| ESP32-CAM thermal overload during streaming | Camera init loop + uncapped stream | Added frame rate cap in stream loop; tuned JPEG quality |
| Robot collapses after several walking steps | Voltage sag on servo rail during peak draw | Added capacitors across servo power rails on expansion shield |
| Serial command pileup / missed commands | Arduino Nano serial buffer overflow | Investigate buffer flushing and command acknowledgement in Nano firmware |

---

## Project Status

- [x] Power system designed and tested
- [x] ESP32-CAM video streaming working
- [x] Serial command relay (ESP32-CAM → Arduino Nano) working
- [x] Basic walking gait implemented
- [x] Thermal fix applied (firmware)
- [x] Servo rail capacitors added (hardware)
- [ ] Serial buffer/command pileup — under investigation
- [ ] Tuned walking gait for stable multi-step locomotion
- [ ] Dashboard UI polish
- [ ] OTA firmware update support
