# Firebird V — Autonomous Face Tracking Robot with Obstacle Avoidance

> **Platform:** Firebird V (ATmega2560) + Raspberry Pi 5  
> **Vision:** YOLOv8n-face · OpenCV · MJPEG streaming  
> **Interface:** Flask web dashboard (live telemetry + voice control)  
> **Communication:** UART2 @ 115200 baud (Pi → ATmega)
> **Watch the demo on YouTube:** [![Watch the demo](https://img.youtube.com/vi/VIDEO_ID/maxresdefault.jpg)](https://youtu.be/TOKNqYgzQEI)

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Setup](#3-hardware-setup)
4. [Software & Tools Used](#4-software--tools-used)
5. [ATmega2560 Firmware](#5-atmega2560-firmware)
   - 5.1 [Motor Control](#51-motor-control)
   - 5.2 [UART Command Protocol](#52-uart-command-protocol)
   - 5.3 [Anti-Spin Turning Logic](#53-anti-spin-turning-logic)
   - 5.4 [Sharp IR Distance Sensing](#54-sharp-ir-distance-sensing)
   - 5.5 [Obstacle Avoidance State Machine](#55-obstacle-avoidance-state-machine)
   - 5.6 [Watchdog Timer](#56-watchdog-timer)
   - 5.7 [LCD Display](#57-lcd-display)
6. [Raspberry Pi 5 — Python Tracker](#6-raspberry-pi-5--python-tracker)
   - 6.1 [Camera Pipeline](#61-camera-pipeline)
   - 6.2 [Face Detection with YOLOv8](#62-face-detection-with-yolov8)
   - 6.3 [Face Lock Logic](#63-face-lock-logic)
   - 6.4 [Proportional Steering](#64-proportional-steering)
   - 6.5 [Serial Debounce](#65-serial-debounce)
   - 6.6 [Voice Control with Vosk](#66-voice-control-with-vosk)
7. [Dashboard](#7-dashboard)
8. [Replication Guide — Step by Step](#8-replication-guide--step-by-step)
9. [Calibration & Tuning](#9-calibration--tuning)
10. [Known Issues & Fixes Applied](#10-known-issues--fixes-applied)
11. [Project File Structure](#11-project-file-structure)

---

## 1. Project Overview

This project converts a standard Firebird V differential-drive robot into a person-following robot. A USB webcam mounted on the robot streams video to a co-processor (Raspberry Pi 5). The Pi runs a real-time face detection pipeline, computes a horizontal error signal, and sends single-byte motion commands over UART to the ATmega2560 microcontroller.

The ATmega2560 controls the DC motors via PWM and independently monitors a Sharp IR distance sensor on the front. If an obstacle is detected within the configured safe distance, the firmware overrides the Pi's commands and executes a pre-programmed avoidance maneuver (reverse → left turn → wall-follow → safety forward → right turn → resume tracking), then returns control to the Pi.

A browser-accessible Flask dashboard streams the annotated live video feed alongside real-time telemetry: face confidence, tracking offset, current motor command, system temperature, RAM usage, UART link status, and a Vosk offline voice command terminal.

---

## 2. System Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                          │
│                                                            │
│  ┌──────────────┐    ┌──────────────┐    ┌─────────────┐  │
│  │ ThreadedCam  │───▶│  YOLOv8n     │───▶│  FaceLock   │  │
│  │  (OpenCV)    │    │  inference   │    │  (tracking) │  │
│  └──────────────┘    └──────────────┘    └──────┬──────┘  │
│                                                 │          │
│                                         decide_command()   │
│                                                 │          │
│  ┌──────────────┐                       ┌──────▼──────┐   │
│  │  VoskThread  │──────────────────────▶│ RobotSerial │   │
│  │  (offline    │   voice override      │  (debounce) │   │
│  │   ASR)       │                       └──────┬──────┘   │
│  └──────────────┘                              │           │
│                                         UART2 @ 115200     │
│  ┌────────────────────────────────┐            │           │
│  │  Flask App                     │            │           │
│  │  /           → dashboard HTML  │            │           │
│  │  /video_feed → MJPEG stream    │            │           │
│  │  /status     → JSON telemetry  │            │           │
│  └────────────────────────────────┘            │           │
└────────────────────────────────────────────────│───────────┘
                                                 │
                                    ┌────────────▼────────────┐
                                    │    ATmega2560            │
                                    │                          │
                                    │  UART2 RX ISR            │
                                    │  → committed_cmd         │
                                    │                          │
                                    │  PRIORITY LOOP:          │
                                    │  1. 'S' → stop all       │
                                    │  2. Avoidance running    │
                                    │  3. Front IR blocked     │
                                    │  4. L/R turns (anti-spin)│
                                    │  5. F → forward          │
                                    │                          │
                                    │  Sharp GP2D12 (front)    │
                                    │  Proximity IR (right)    │
                                    │  Motor PWM (Timer5)      │
                                    │  16x2 LCD (PORTC)        │
                                    └──────────────────────────┘
```

---

## 3. Hardware Setup

| Component | Details |
|---|---|
| Robot chassis | Firebird V (e-Yantra / ERTS Lab) |
| MCU | ATmega2560 @ 14.7456 MHz crystal |
| Co-processor | Raspberry Pi 5 (4 GB / 8 GB) |
| Camera | USB webcam (any UVC-compliant) |
| Front distance sensor | Sharp GP2D12 / GP2Y0A21 (10–80 cm) |
| Right proximity sensor | Onboard IR proximity (Firebird V, ADC ch 6 or 7) |
| Motor driver | L293D (onboard Firebird V) |
| UART connection | Pi USB-to-UART adapter → ATmega UART2 (PH0/PH1) |
| Power | 7.4 V LiPo → onboard regulator → 5 V rails |

### Wiring — Pi to ATmega UART2

```
Pi USB-UART adapter TX  →  ATmega2560 PH0 (RXD2)
Pi USB-UART adapter RX  →  ATmega2560 PH1 (TXD2)
GND                     →  GND (common)
```

> The ATmega's UART2 is not the same as UART0 (used for USB-to-PC debugging).
> UART2 maps to PH0/PH1, enabled via `UCSR2B`.

---

## 4. Software & Tools Used

| Layer | Tool / Library | Version / Notes |
|---|---|---|
| ATmega firmware IDE | **Atmel Studio 7.0** | AVR-GCC toolchain, ISP via onboard programmer |
| AVR libraries | `avr/io.h`, `avr/interrupt.h`, `avr/wdt.h`, `util/delay.h` | Part of AVR-Libc |
| Python runtime | **Python 3.11+** | Runs on Pi OS (64-bit) |
| Object detection | **Ultralytics YOLOv8** (`ultralytics` pip package) | Model: `yolov8n-face-lindevs.pt` |
| Computer vision | **OpenCV** (`opencv-python`) | Frame capture, drawing, JPEG encode |
| Serial | **pyserial** | UART bridge to ATmega |
| Web framework | **Flask** | Serves dashboard + MJPEG stream + `/status` JSON |
| Offline ASR | **Vosk** + `sounddevice` | `vosk-model-small-en-in-0.4` (or `en-us`) |
| Dashboard UI | Vanilla HTML + Tailwind CSS (CDN) | No build step required |
| System stats | `psutil` (optional) / `/proc/meminfo` fallback | CPU temp from `/sys/class/thermal/thermal_zone0/temp` |

---

## 5. ATmega2560 Firmware

The firmware is written in pure C (no Arduino framework) and compiled in **Atmel Studio 7.0** with AVR-GCC. The main loop runs with a 1 ms software tick (`_delay_ms(1)` + `g_ms++`) to derive non-blocking timing without relying on hardware Timer0 (which is occupied by the motor PWM on Timer5).

### 5.1 Motor Control

Motor direction is set via **PORTA bits [3:0]** which map to the two H-bridge inputs:

```c
static inline void motor_forward(void)  { PORTA = (PORTA & 0xF0) | 0x06; }
static inline void motor_back(void)     { PORTA = (PORTA & 0xF0) | 0x09; }
static inline void motor_left(void)     { PORTA = (PORTA & 0xF0) | 0x05; } // zero-radius left
static inline void motor_right(void)    { PORTA = (PORTA & 0xF0) | 0x0A; } // zero-radius right
static inline void motor_stop(void)     { PORTA = (PORTA & 0xF0) | 0x00; }
```

Speed (PWM duty cycle) is controlled via **Timer5** (`TCCR5A = 0xA9`, `TCCR5B = 0x0B`) which drives OCR5AL (left motor) and OCR5BL (right motor). Three speed presets are used:

| Preset | OCR value | Used for |
|---|---|---|
| `set_full_speed()` | 255 | Forward tracking |
| `set_turn_speed()` | 170 | Hard L/R turns + avoidance turns |
| `set_soft_turn_speed()` | 120 | Soft l/r corrections from Pi |
| `set_avoid_speed()` | 200 | Reverse + wall-follow phases |

### 5.2 UART Command Protocol

The Pi sends a **single ASCII byte** per command. The ATmega's UART2 receive-complete ISR reads it and validates:

```c
ISR(USART2_RX_vect) {
    uint8_t b = UDR2;
    if(b=='F'||b=='L'||b=='R'||b=='l'||b=='r'||b=='S'||b=='B') {
        g_pi_cmd  = b;
        g_cmd_new = 1;
    }
    wdt_reset();
}
```

| Byte | Meaning |
|---|---|
| `F` | Full forward |
| `L` | Hard left (zero-radius) |
| `R` | Hard right (zero-radius) |
| `l` | Soft left (face slightly off-center left) |
| `r` | Soft right (face slightly off-center right) |
| `S` | Stop (highest priority — cancels everything) |

The main loop copies `g_pi_cmd` into `committed_cmd` each iteration. Using a committed copy (rather than reading `g_pi_cmd` directly across multiple priority checks) prevents a race condition where the ISR fires between two reads of the same variable in a single loop pass.

### 5.3 Anti-Spin Turning Logic

Early testing showed that Pi continuously sending `L` or `R` while the face drifts causes the robot to spin indefinitely — the camera view rotates faster than the face can re-center, overshooting in the opposite direction, then spinning back. This creates an oscillating spin cycle.

The fix is a **burst-and-pause** pattern implemented in the main loop:

```
Turn burst (300 ms at turn speed)
  → Motor stop
  → Pause (150 ms at zero speed, full PWM ready)
  → Repeat if Pi still sending same turn command
```

For soft turns (`l`/`r`) the burst is shorter (120 ms) and speed is lower (120 PWM) to produce gentle corrections without upsetting tracking lock.

The critical implementation detail: the `turning` and `in_pause` flags must be **reset to 0 whenever the avoidance state machine takes over**, and when an `S` command arrives, otherwise stale timing state can cause the robot to re-enter a turn mid-avoidance or re-enter a pause after resuming forward.

### 5.4 Sharp IR Distance Sensing

The Sharp GP2D12 outputs an analog voltage inversely proportional to distance. The ATmega's ADC reads it on **channel 11** (PORTK, using the extended multiplexer via `ADCSRB = 0x08`):

```c
uint16_t ir_distance_cm(void) {
    uint8_t raw = adc_read(FRONT_SENSOR_CHANNEL);
    if(raw < 3) raw = 3;   // clamp — avoid division by near-zero
    float mm = 10.0f * (2799.6f * powf((float)raw, -1.1546f));
    uint16_t cm = (uint16_t)(mm / 10.0f);
    return (cm > 80) ? 80 : cm;   // sensor saturates beyond ~80 cm
}
```

The conversion formula `d_mm = 2799.6 × raw^(−1.1546)` is the datasheet power-law curve-fit for the GP2D12 (valid for raw ADC 10–200). Values below 3 are clamped because the formula diverges to infinity near zero. Values above 80 cm are saturated because the sensor is physically unreliable beyond that range.

**ADC configuration:**

- `ADMUX = 0x20` — left-adjust result, use AVCC as reference
- `ADCSRA = 0x86` — ADC enable, prescaler /64 (gives ~230 kHz ADC clock at 14.7 MHz crystal, within the 50–200 kHz spec)
- Result is read from `ADCH` only (8-bit left-adjusted), which is sufficient for this sensor's accuracy

### 5.5 Obstacle Avoidance State Machine

When the front IR reads ≤ 20 cm (`IR_STOP_CM`), the firmware overrides all Pi commands and runs a deterministic avoidance maneuver. This maneuver assumes the robot is in a corridor-like environment where a wall exists to the right, and bypasses the obstacle using a left deviation.

```
STATE_TRACKING
    │
    │  front IR <= 20 cm
    ▼
STATE_FREEZE_REVERSE ──── reverse until avg encoder ticks >= TICKS_REVERSE
    │
    ▼
STATE_TURN_90_LEFT ──── turn left until right encoder ticks >= TICKS_90_DEG
    │
    ▼
STATE_WALL_FOLLOW ──── forward until right proximity ADC < RIGHT_IR_WALL_THRES
    │                  (right wall disappears = obstacle cleared)
    ▼
STATE_SAFETY_FWD ──── forward until avg encoder ticks >= TICKS_SAFETY_FWD
    │                 (clears the robot's own body past the obstacle edge)
    ▼
STATE_TURN_90_RIGHT ──── turn right until left encoder ticks >= TICKS_90_DEG
    │
    ▼
STATE_CHECK_CLEAR ──── re-check front IR
    │   if clear  → STATE_TRACKING  (resume Pi control)
    │   if blocked → STATE_FREEZE_REVERSE  (retry)
    └─────────────────────────────────────────────────────────┘
```

**Why encoder-based instead of time-based?**

An earlier version of the firmware used fixed `_delay_ms()` durations for each phase (400 ms reverse, 450 ms turn, etc.). This was fragile: battery voltage drop changed motor speed, so the same delay produced different arc lengths as the battery drained. Switching to wheel encoder tick counts means the robot travels a consistent distance regardless of speed.

The encoder signals come from optical shaft encoders on PE4 (INT4, left motor) and PE5 (INT5, right motor), triggered on falling edge:

```c
ISR(INT4_vect) { left_shaft_count++; }
ISR(INT5_vect) { right_shaft_count++; }
```

For the 90° turn, only the **outer wheel's** encoder is counted (right wheel for left turn, left wheel for right turn) because the inner wheel is nearly stationary in a zero-radius turn — counting the average would undercount the actual rotation.

**Priority ordering** in the main loop is critical. Checked top-to-bottom every iteration:

1. `'S'` from Pi → immediate stop, reset all state
2. Avoidance state machine running → execute current phase, ignore Pi
3. Front IR blocked → trigger avoidance (only if not already avoiding)
4. Pi L/R/l/r → anti-spin turns
5. Pi F → forward
6. Anything else → stop

### 5.6 Watchdog Timer

The WDT is configured for a 1-second timeout (`WDTO_1S`). It is reset in two places:

- `wdt_reset()` at the top of every main loop iteration
- Inside `ISR(USART2_RX_vect)` when valid bytes arrive

If the Pi crashes, stops sending, or the UART link drops, the ATmega will reset itself after 1 second. On reset it powers up in stop state (`committed_cmd = 'S'`), so the robot will halt rather than continue moving blindly. The line `MCUSR = 0; wdt_disable();` at the start of `main()` clears any stale reset-cause flags before re-enabling the WDT.

### 5.7 LCD Display

A 16×2 HD44780-compatible LCD is connected in **4-bit mode** via PORTC. The firmware uses a custom software 4-bit driver (no external library). Display is updated every ~200 main loop iterations (~200 ms) to avoid spending excessive CPU time on slow LCD writes.

Row 1 shows live sensor readings and the last Pi command:

```
IR:38cm  Pi:F
```

Row 2 shows the current operational state:

```
FORWARD
AVOID: REVERSE
ANTI-SPIN PAUSE
```

---

## 6. Raspberry Pi 5 — Python Tracker

### 6.1 Camera Pipeline

A standard `cv2.VideoCapture` object is wrapped in a `ThreadedCamera` class that reads frames on a **dedicated background thread** and stores the latest frame in a shared buffer with a `threading.Lock`. The main tracker loop calls `cam.read()` which returns a copy of whatever the background thread last captured.

This decouples the camera capture rate from the inference rate. Without threading, `cap.read()` blocks until the next frame arrives (often 30–40 ms at 30 fps), stalling the whole tracker. With threading, `cam.read()` returns instantly with the most recent frame, and the inference pipeline runs as fast as the hardware allows.

**Capture configuration:**

```python
FRAME_W = 640
FRAME_H = 480
```

Capture at 640×480 gives acceptable visual quality for the MJPEG stream without overwhelming the Pi's USB bandwidth. The YOLO inference internally resizes to `imgsz=320` for speed — this is independent of the capture resolution and only affects the detection accuracy/speed tradeoff.

### 6.2 Face Detection with YOLOv8

The project uses `yolov8n-face-lindevs.pt` — a nano-size YOLOv8 model fine-tuned specifically for face detection. If this file is missing, the code falls back to `yolov8n.pt` (general object detection, `class_id == 0` = person).

**Why YOLOv8n-face instead of alternatives?**

- Haar cascades (`haarcascade_frontalface`) are fast but have high false-positive rates on complex backgrounds and non-frontal faces — a real problem in a lab corridor with multiple people
- General YOLOv8n detects persons at body level, not face level — the bounding box centroid is at body center, not the face, which gave worse horizontal tracking accuracy for a narrow camera FOV
- YOLOv8n-face detects face bounding boxes directly, giving a centroid that accurately represents where the person's head is
- Nano size runs at ~30 FPS on Pi 5 with `imgsz=320` without any hardware accelerator

Inference call:

```python
results = model(frame, imgsz=320, conf=CONF_THRESHOLD, verbose=False)
```

`verbose=False` suppresses per-frame stdout logging which otherwise creates significant I/O overhead and slows the loop measurably.

### 6.3 Face Lock Logic

When multiple faces are in frame, the tracker must follow a consistent target rather than jumping between faces each frame. The `FaceLock` class implements this:

**Initial lock:** when no face is currently locked, pick the detection with the highest confidence score.

**Maintaining lock:** on subsequent frames, among all current detections, pick the one whose centroid is closest (Euclidean distance) to the last locked centroid. This handles the case where confidence scores fluctuate — the spatially nearest box is the most likely continuation of the same face.

**Grace period:** if the locked face disappears (person walked away, occlusion, detection miss), the tracker does not immediately command stop. It holds the last good command for up to 60 frames (~2 seconds at 30 fps). If the face reappears within this window, tracking resumes seamlessly. If not, the robot stops and the lock is released.

### 6.4 Proportional Steering

The horizontal error is the signed difference between the detected face centroid and the frame center:

```python
error = target_cx - frame_cx   # positive = face is to the right of center
```

A three-zone scheme maps this error to a command:

```
|error| <= CENTER_TOL (150 px)              →  F   (forward, face centered)
CENTER_TOL < |error| <= TOL + PROP (230 px) →  r/l (soft correction)
|error| > TOL + PROP                        →  R/L (hard turn)
```

**Why three zones instead of continuous PWM?**

The ATmega receives single-character commands and implements its own PWM presets. Sending continuous speed values over UART would require a more complex framing protocol (multi-byte, parsing, error detection). The three-zone approach gives proportional-like behavior using the existing simple character protocol. The ATmega's anti-spin burst logic further smooths the motion.

`CENTER_TOL = 150 px` at 640 wide = 23% of frame width per side — wide enough that the robot ignores minor head movements and detection jitter, but responsive enough to track large offsets promptly.

### 6.5 Serial Debounce

Raw tracking sends the same command many consecutive times (e.g., 30 `F` bytes per second). Writing every command wastes UART bandwidth and can cause buffer buildup on the ATmega. The `RobotSerial.send()` method applies two rules:

1. **Debounce N:** a command must be seen `DEBOUNCE_N = 3` times consecutively before it is transmitted. This filters single-frame noise detections.
2. **Rate limit:** even if the command is stable, it is only written to serial once per `CMD_INTERVAL = 0.08` seconds (12.5 Hz maximum).

Combined: a spurious single-frame `L` detection does not trigger a turn. A sustained 3-frame `L` detection does, but only after the minimum interval has elapsed.

### 6.6 Voice Control with Vosk

Vosk provides **fully offline speech recognition** — no internet required, no API key, runs entirely on the Pi. This is important for lab environments without reliable WiFi and where cloud ASR latency would make voice control feel unresponsive.

The recognition vocabulary is deliberately restricted to just two words:

```python
grammar = '["start", "stop", "[unk]"]'
```

Restricting the vocabulary via Vosk's `KaldiRecognizer` grammar parameter dramatically improves recognition accuracy for the target words and reduces the chance of ambient speech accidentally triggering commands.

**Runtime flow:**

- `sounddevice.RawInputStream` captures 16 kHz, 16-bit mono audio continuously on a background thread
- Partial results are polled on every audio block for low-latency detection (rather than waiting for `AcceptWaveform` to confirm a complete utterance)
- On detecting `"start"`: sets `robot_active = True` — robot resumes executing Pi tracking commands
- On detecting `"stop"`: sets `robot_active = False`, sends `'S'` to ATmega immediately — robot halts regardless of what the tracker is computing
- `rec.Reset()` is called immediately after each recognized command to flush the acoustic buffer and prevent re-triggering

When `robot_active = False`, the tracker continues computing commands internally (FaceLock runs, error is computed) but `robot_instance.send("S")` is called unconditionally. This means the robot responds instantly when voice re-enables it — no warmup lag.

---

## 7. Dashboard

The dashboard is a single-page no-scroll HTML file served directly by Flask at `/`. It uses Tailwind CSS from CDN (no build step) and plain JavaScript.

### Layout

```
+----------------------------------+------------------+
|                                  |  AI Vision State |
|         Live Video Feed          |  (lock + conf)   |
|         /video_feed MJPEG        +------------------+
|                                  |  Kinematics      |
|  YOLOv8 bboxes drawn by Pi:      |  (current cmd)   |
|  green = all detections          +------------------+
|  red   = locked target           |  System Diag     |
|                                  |  (temp/RAM/UART) |
|  CSS crosshair overlay           +------------------+
|                                  |  Acoustic Term.  |
|  62% width                       |  (Vosk log)      |
+----------------------------------+------------------+
```

### Data Flow

The dashboard JS polls `/status` every **400 ms**. Flask returns a JSON snapshot of `dash_state`, which the tracker loop updates every frame:

```json
{
  "lock_status":  "TARGET ACQUIRED",
  "confidence":   0.624,
  "offset_error": 90,
  "command":      "SOFT RIGHT",
  "robot_active": false,
  "uart_ok":      true,
  "cpu_temp":     63.9,
  "ram_pct":      12.7,
  "terminal": [
    {"text": "acoustic stream connected", "type": "info"},
    {"text": "> start", "type": "heard"},
    {"text": "OVERRIDE: SYSTEM ACTIVE", "type": "override"}
  ]
}
```

The video stream (`/video_feed`) and telemetry (`/status`) are on separate routes so they update at different rates without blocking each other.

### Design Decisions

- **Monochrome palette:** strictly black / white / muted gray — the live feed already adds color (green/red bounding boxes); adding color to the widgets would compete visually
- **No scroll:** the entire dashboard fits `100vh` via CSS Grid; right-column widgets use `flex: 1` to share height equally
- **Bracket corners:** CSS `::before`/`::after` pseudo-elements draw the corner brackets without extra DOM elements
- **Kinematics bar:** three `div` bars (L / center / R) flip class to white/muted to indicate direction at a glance, faster to read than the text label alone
- **400 ms poll interval:** fast enough to feel live, slow enough that `/status` JSON responses do not compete meaningfully with the MJPEG stream for CPU time on the Pi

---

## 8. Replication Guide — Step by Step

### Step 1 — Flash the ATmega2560 Firmware

1. Open **Atmel Studio 7.0**
2. Create a new GCC C Executable Project, target device `ATmega2560`
3. Paste `main.c` from this repo
4. Verify crystal frequency: `#define F_CPU 14745600UL`
   - Check the crystal on your board — some Firebird V units ship with 11.0592 MHz
   - If different, recalculate: `UBRR_VAL = (F_CPU / (16 * BAUD)) - 1`
5. Build → Program via onboard ISP (USBasp / STK500 compatible)

### Step 2 — Set Up the Raspberry Pi 5

```bash
# System dependencies
sudo apt update
sudo apt install python3-pip python3-opencv libportaudio2

# Python packages
pip install ultralytics pyserial flask vosk sounddevice psutil

# Download Vosk model
wget https://alphacephei.com/vosk/models/vosk-model-small-en-in-0.4.zip
unzip vosk-model-small-en-in-0.4.zip -d model
# The 'model/' directory must be in the same folder as app.py

# Download YOLOv8 face model
wget https://github.com/lindevs/yolov8-face/releases/latest/download/yolov8n-face-lindevs.pt
```

### Step 3 — Connect Pi to ATmega

Connect a USB-UART bridge (CH340, CP2102, or FT232-based) between a Pi USB port and the ATmega2560 UART2 header (PH0/PH1) on the Firebird V expansion connector. Ensure common ground.

Verify the port appears:

```bash
ls /dev/ttyUSB*
# expected: /dev/ttyUSB0
```

### Step 4 — Connect the Camera

Plug a USB webcam into the Pi. Quick test:

```python
import cv2
cap = cv2.VideoCapture(0)
print(cap.read()[0])  # should print True
cap.release()
```

If camera is not at index 0, change `CAMERA_INDEX` in `app.py`.

### Step 5 — Run the Application

```bash
cd /path/to/project
python app.py
```

Expected startup output:

```
[MODEL] Loading yolov8n-face-lindevs.pt...
[MODEL] Loaded.
[CAM] Threaded camera stream initialized.
[SERIAL] Found: /dev/ttyUSB0  (USB Serial)
[SERIAL] Connected on /dev/ttyUSB0 @ 115200 baud
[VOICE] Loading offline acoustic model...
[VOICE] Microphone active. Listening for 'START' or 'STOP'...
[RUN] Active and running.
```

### Step 6 — Open the Dashboard

From any device on the same network:

```
http://<raspberry-pi-ip>:5000
```

Say **"start"** — the Acoustic Terminal widget will show the recognition event and the robot will begin following faces visible to the camera.

---

## 9. Calibration & Tuning

### Tracking Sensitivity

| Parameter | File | Default | Effect of increasing |
|---|---|---|---|
| `CENTER_TOL` | `app.py` | 150 px | Larger dead zone, robot moves less on small offsets |
| `PROP_THRESHOLD` | `app.py` | 80 px | Wider soft-turn zone before hard turn triggers |
| `CONF_THRESHOLD` | `app.py` | 0.35 | Fewer false detections, may miss low-lit faces |
| `DEBOUNCE_N` | `app.py` | 3 | More filtering, slightly more command lag |
| `LOCK_LOST_SEC` | `app.py` | 5.0 s | Longer grace period before lock releases |

### Obstacle Avoidance Distance

| Parameter | File | Default | Effect of increasing |
|---|---|---|---|
| `IR_STOP_CM` | `main.c` | 20 cm | Avoidance triggers earlier, more conservative |

### Avoidance Tick Counts

These depend on your exact wheel diameter and encoder pulses-per-revolution. Calibrate empirically:

```
TICKS_REVERSE    = 15   // ~5 cm reverse clearance
TICKS_90_DEG     = 30   // ~90 degree zero-radius turn
TICKS_SAFETY_FWD = 45   // ~half robot body length forward
```

**Calibration procedure:**
1. Place a tape measure on the floor
2. Temporarily add `printf` output on UART0 to log encoder counts
3. Run each avoidance phase in isolation
4. Count ticks vs measured distance/angle at nominal battery voltage
5. Adjust constants accordingly

### Anti-Spin Timing

| Parameter | File | Default | Reduce if... |
|---|---|---|---|
| `TURN_BURST_MS` | `main.c` | 300 ms | Robot overshoots face on hard turns |
| `PAUSE_MS` | `main.c` | 150 ms | Robot still oscillates after turning |
| `SOFT_TURN_BURST_MS` | `main.c` | 120 ms | Soft corrections are too large |
| `SOFT_PAUSE_MS` | `main.c` | 150 ms | |

---

## 10. Known Issues & Fixes Applied

### Issue 1 — Lowercase soft-turn commands dropped by ISR

**Symptom:** `l` and `r` commands sent by Pi were never executed. Robot only responded to uppercase `L`, `R`, `F`, `S`.

**Root cause:** The ISR validation condition in an early firmware version only checked uppercase characters. Lowercase ASCII bytes arrived and were silently discarded.

**Fix:** Expanded ISR validation:
```c
if(b=='F'||b=='L'||b=='R'||b=='l'||b=='r'||b=='S'||b=='B')
```

### Issue 2 — Robot oscillates / spins continuously during tracking

**Symptom:** When a face is off-center, the robot turns, overcorrects, turns back, producing continuous oscillation.

**Root cause:** Camera FOV is narrow. By the time the ATmega executes a turn and the Pi captures and processes the next frame, the face has moved to the opposite side — the system chased its own overshoot.

**Fix:** Anti-spin burst-pause logic in firmware (§5.3). Tuning `TURN_BURST_MS` down reduces overshoot for a given camera FOV.

### Issue 3 — Avoidance maneuver inconsistent across battery charge levels

**Symptom:** At full charge, 90° turn completed sooner than calibrated. Near empty, robot failed to turn far enough.

**Root cause:** Original time-based avoidance phases (`_delay_ms`) depended on motor speed which varies with supply voltage.

**Fix:** Replaced all time-based phases with wheel encoder tick count termination (§5.5).

### Issue 4 — Video feed heavily blurred in dashboard

**Symptom:** Live feed in dashboard looked like upscaled 320×240 regardless of viewing device.

**Root cause:** `FRAME_W = 320` was the original capture size. JPEG quality was set to 50. The dashboard displays at 62% of a full-screen viewport, heavily upscaling the small frame.

**Fix:**
- `FRAME_W = 640, FRAME_H = 480`
- JPEG quality raised to 85
- `CENTER_TOL` and `PROP_THRESHOLD` scaled 2× proportionally to compensate for the wider frame

### Issue 5 — Vertical colored lines across video feed in dashboard

**Symptom:** Three persistent vertical lines (white center, two pink/red tolerance band markers) visible through the entire live feed in the dashboard.

**Root cause:** These were `cv2.line()` debug overlays drawn directly into the frame pixels inside the tracker loop — carried over from standalone Flask testing where they served as visual reference. They were baked into the JPEG before streaming; they were not a CSS effect.

**Fix:** Removed all three `cv2.line()` calls. The offset error is now shown numerically in the dashboard "Offset Error" widget, which is more precise than a visual line overlay.

---

## 11. Project File Structure

```
project/
├── app.py                     # Pi main: tracker + voice + Flask server
├── yolov8n-face-lindevs.pt   # YOLOv8 nano face detection model weights
├── model/                     # Vosk offline ASR model directory
│   └── ...                    # (extracted from downloaded zip)
├── firmware/
│   └── main.c                 # ATmega2560 firmware (Atmel Studio 7.0)
└── README.md                  # This file
```

---

## Authors

Developed as an internship project at **KPIT Apex Lab**.  
Platform: COEP Technological University, Pune — Electronics & Telecommunication Engineering.  
Stack: AVR-C bare-metal firmware · Python computer vision pipeline · Flask web dashboard.

## Team Members

- Darpan Dongre
- Pratham Modi
- Amay Bembde

---

## Internship

Developed as part of the **KPIT APEX Lab Internship**.
