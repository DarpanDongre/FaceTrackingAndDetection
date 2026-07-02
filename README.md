# Firebird V Autonomous Human Tracking Robot

An autonomous human tracking robot developed as part of the **KPIT APEX Lab Internship**. The system combines computer vision, offline speech recognition, and embedded motor control to enable the Firebird V robot to detect, lock onto, and follow a human target in real time while avoiding obstacles.

---

## Overview

The Raspberry Pi performs real-time computer vision using YOLOv8 to detect people and faces from a USB webcam. Once a target is detected, the robot locks onto the person's face and continuously tracks their position. Movement commands are generated based on the target's position in the camera frame and transmitted to the Firebird V's ATmega microcontroller over UART. The robot also supports offline voice commands for hands-free operation and features a live web dashboard for monitoring system status.

---

## Features

- Real-time human detection using YOLOv8
- Face Lock for stable target tracking
- Autonomous human following
- Obstacle avoidance
- Offline voice commands using Vosk
- Live web dashboard
- UART communication between Raspberry Pi and ATmega
- Low-latency threaded camera capture
- Automatic serial port detection
- Proportional steering for smooth robot movement

---

## Hardware

- Firebird V Robot
- Raspberry Pi
- ATmega Microcontroller
- USB Webcam
- Distance Sensors
- DC Motors

---

## Software

- Python 3
- OpenCV
- Ultralytics YOLOv8
- Flask
- PySerial
- Vosk Speech Recognition
- SoundDevice
- Atmel Studio

---

## System Architecture

```
USB Camera
      │
      ▼
 Raspberry Pi
      │
YOLOv8 Detection
      │
Face Lock
      │
Tracking Logic
      │
Obstacle Avoidance
      │
Voice Command Override
      │
UART
      │
      ▼
ATmega Controller
      │
Motor Driver
      │
      ▼
Firebird V Robot
```

---

## How It Works

1. The USB webcam continuously captures video.
2. YOLOv8 detects humans (and faces when available).
3. The system locks onto the detected face.
4. Tracking error is calculated relative to the center of the frame.
5. Proportional steering generates smooth movement commands.
6. Obstacle avoidance ensures safe navigation.
7. Commands are transmitted over UART to the Firebird V's ATmega controller.
8. The ATmega drives the motors to follow the target.
9. Offline voice commands can activate or stop the robot at any time.
10. A Flask dashboard displays live video, tracking status, system diagnostics, and command telemetry.

---

## Team Members

- Darpan Dongre
- Pratham Modi
- Amay Bembde

---

## Internship

Developed as part of the **KPIT APEX Lab Internship**.
