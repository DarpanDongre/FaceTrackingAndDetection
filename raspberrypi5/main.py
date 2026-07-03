import time
import sys
import threading
import queue
import json
from pathlib import Path
from collections import deque

import cv2
import serial
import serial.tools.list_ports
from flask import Flask, Response, render_template_string, jsonify
from ultralytics import YOLO
import sounddevice as sd
from vosk import Model, KaldiRecognizer

try:
    import psutil
    HAVE_PSUTIL = True
except Exception:
    HAVE_PSUTIL = False

# ── CONFIG ────────────────────────────────────────────────────────────────
FRAME_W           = 640   # capture/stream resolution — was 320, doubled for sharper feed
FRAME_H           = 480   # was 240
CENTER_TOL        = 150   # scaled 2x to match FRAME_W (was 75 @ 320 wide)
PROP_THRESHOLD    = 80    # scaled 2x to match FRAME_W (was 40 @ 320 wide)
BAUD_RATE         = 115200
CMD_INTERVAL      = 0.08
DEBOUNCE_N        = 3
CONF_THRESHOLD    = 0.35
CAMERA_INDEX      = 0
STREAM_FPS_DELAY  = 0.01

LOCK_LOST_SEC     = 3.0

MODEL_FACE        = "yolov8n-face-lindevs.pt"
MODEL_PERSON      = "yolov8n.pt"

app               = Flask(__name__)
latest_frame      = None
latest_frame_lock = threading.Lock()

robot_active   = False
robot_instance = None

# ── LIVE DASHBOARD STATE (read by /status, written by the tracker + voice threads) ──
dash_lock  = threading.Lock()
dash_state = {
    "lock_status":  "SEARCHING...",
    "confidence":   None,
    "offset_error": 0,
    "command":      "STOP",
    "robot_active": False,
    "uart_ok":      False,
    "cpu_temp":     None,
    "ram_pct":      None,
    "terminal":     [],     # list of {"text":..., "type": "info"|"heard"|"override"}
}
terminal_log = deque(maxlen=18)

CMD_NAME = {
    "F": "FORWARD",
    "L": "LEFT",
    "R": "RIGHT",
    "l": "SOFT LEFT",
    "r": "SOFT RIGHT",
    "S": "STOP",
}

def push_terminal(text, kind="info"):
    terminal_log.append({"text": text, "type": kind})
    with dash_lock:
        dash_state["terminal"] = list(terminal_log)

def update_dash(**kwargs):
    with dash_lock:
        dash_state.update(kwargs)

def read_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None

def read_ram_pct():
    if HAVE_PSUTIL:
        try:
            return round(psutil.virtual_memory().percent, 1)
        except Exception:
            return None
    try:
        meminfo = {}
        with open("/proc/meminfo") as f:
            for line in f:
                k, v = line.split(":")
                meminfo[k.strip()] = int(v.strip().split()[0])
        total = meminfo.get("MemTotal", 0)
        avail = meminfo.get("MemAvailable", 0)
        if total:
            return round((1 - avail / total) * 100, 1)
    except Exception:
        pass
    return None

def system_stats_thread():
    while True:
        update_dash(cpu_temp=read_cpu_temp(), ram_pct=read_ram_pct())
        time.sleep(2)

# ── ULTRA-FAST THREADED CAMERA BACKGROUND GRABBER ──────────────────────────
class ThreadedCamera:
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.ret, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = threading.Thread(target=self.update, args=(), daemon=True)
        self.thread.start()
        return self

    def update(self):
        while self.started:
            ret, frame = self.cap.read()
            if ret:
                with self.read_lock:
                    self.ret = ret
                    self.frame = frame
            time.sleep(0.01)

    def read(self):
        with self.read_lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def release(self):
        self.started = False
        if hasattr(self, "thread"):
            self.thread.join()
        self.cap.release()

# ── AUTO-DETECT SERIAL PORT ──────────────────────────────────────────────
def find_firebird_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if "usb" in desc or "uart" in desc or "ch340" in desc or "cp210" in desc or "ft232" in desc:
            print(f"[SERIAL] Found: {p.device}  ({p.description})")
            return p.device
    print("[SERIAL] No USB-UART found by name, defaulting to /dev/ttyUSB0")
    return "/dev/ttyUSB0"

# ── SERIAL WRITE WITH DEBOUNCE ───────────────────────────────────────────
class RobotSerial:
    def __init__(self, port, baud):
        self.ser           = serial.Serial(port, baud, timeout=0.1)
        self.last_sent     = b"S"
        self.pending       = b"S"
        self.pending_count = 0
        self.last_time     = 0.0
        time.sleep(2)
        print(f"[SERIAL] Connected on {port} @ {baud} baud")
        update_dash(uart_ok=True)

    def send(self, cmd_char: str):
        now = time.time()
        cmd = cmd_char.encode()

        if cmd == self.pending:
            self.pending_count += 1
        else:
            self.pending       = cmd
            self.pending_count = 1

        if self.pending_count >= DEBOUNCE_N and (now - self.last_time) >= CMD_INTERVAL:
            if cmd != self.last_sent:
                self.ser.write(cmd)
                self.last_sent = cmd
                self.last_time = now
            else:
                self.ser.write(cmd)
                self.last_time = now

    def close(self):
        try:
            self.ser.write(b"S")
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        update_dash(uart_ok=False)

# ── FACE LOCK ─────────────────────────────────────────────────────────────
class FaceLock:
    def __init__(self, lost_timeout=LOCK_LOST_SEC):
        self.locked_box  = None
        self.lost_since  = None
        self.timeout     = lost_timeout

    def update(self, boxes):
        now = time.time()

        if not boxes:
            if self.locked_box is not None:
                if self.lost_since is None:
                    self.lost_since = now
                elif (now - self.lost_since) > self.timeout:
                    self.locked_box = None
                    self.lost_since = None
            return None, False

        self.lost_since = None

        if self.locked_box is None:
            self.locked_box = max(boxes, key=lambda b: b[4])
            return self.locked_box, True

        lx1, ly1, lx2, ly2 = self.locked_box[:4]
        lcx = (lx1 + lx2) / 2
        lcy = (ly1 + ly2) / 2

        best      = None
        best_dist = float("inf")
        for b in boxes:
            bx1, by1, bx2, by2 = b[:4]
            bcx = (bx1 + bx2) / 2
            bcy = (by1 + by2) / 2
            d   = ((bcx - lcx) ** 2 + (bcy - lcy) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best      = b

        self.locked_box = best
        return best, True

# ── PROPORTIONAL STEERING ────────────────────────────────────────────────
def decide_command(error):
    abs_err = abs(error)

    if abs_err <= CENTER_TOL:
        return "F"

    slow_limit = CENTER_TOL + PROP_THRESHOLD

    if abs_err <= slow_limit:
        return "r" if error > 0 else "l"
    else:
        return "R" if error > 0 else "L"

# ── VOICE LISTENER THREAD ────────────────────────────────────────────────
def voice_listener_thread():
    global robot_active
    global robot_instance

    print("[VOICE] Loading offline acoustic model...")
    try:
        model = Model("model")
    except Exception as e:
        print("[VOICE ERROR] Could not load Vosk model.")
        push_terminal("acoustic model load failed", "info")
        return

    q = queue.Queue()

    def callback(indata, frames, time_, status):
        if status:
            print(status, file=sys.stderr)
        q.put(bytes(indata))

    try:
        with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                               channels=1, callback=callback):

            grammar = '["start", "stop", "[unk]"]'
            rec = KaldiRecognizer(model, 16000, grammar)

            print("[VOICE] Microphone active. Listening for 'START' or 'STOP'...")
            push_terminal("acoustic stream connected", "info")
            push_terminal("listening for wake word...", "info")

            while True:
                data = q.get()

                if rec.AcceptWaveform(data):
                    res = json.loads(rec.Result())
                    text = res.get("text", "").lower()
                else:
                    res = json.loads(rec.PartialResult())
                    text = res.get("partial", "").lower()

                if not text:
                    continue

                if "start" in text and not robot_active:
                    print("\n>>> [VOICE] START SYSTEM ACTIVE <<<\n")
                    robot_active = True
                    update_dash(robot_active=True)
                    push_terminal("> start", "heard")
                    push_terminal("OVERRIDE: SYSTEM ACTIVE", "override")
                    rec.Reset()

                elif "stop" in text and robot_active:
                    print("\n>>> [VOICE] STOP SYSTEM HALTED <<<\n")
                    robot_active = False
                    update_dash(robot_active=False)
                    if robot_instance is not None:
                        robot_instance.send("S")
                    push_terminal("> stop", "heard")
                    push_terminal("OVERRIDE: SYSTEM HALTED", "override")
                    rec.Reset()

    except Exception as e:
        print(f"[VOICE ERROR] Microphone issue: {e}")
        push_terminal("microphone error", "info")

# ── STREAM ROUTES ────────────────────────────────────────────────────────
def generate_frames():
    global latest_frame
    while True:
        frame = None
        with latest_frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
        if frame is None:
            time.sleep(0.02)
            continue

        ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ret:
            time.sleep(0.01)
            continue
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buffer.tobytes() +
            b"\r\n"
        )
        time.sleep(STREAM_FPS_DELAY)

@app.route("/")
def index():
    return render_template_string(DASHBOARD_HTML)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/status")
def status():
    with dash_lock:
        return jsonify(dict(dash_state))

# ── DASHBOARD HTML ──
DASHBOARD_HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Firebird V - Stealth Command</title>
    <style>
        /* ── offline replacement for the Tailwind CDN utility classes used below ──
           No external requests — everything the dashboard needs is defined here
           so it works with zero internet connection. Values match the original
           Tailwind config 1:1 (colors, spacing, font sizes etc.) */

        * { box-sizing: border-box; }

        html, body { margin: 0; padding: 0; }

        body {
            background-color: #000;
            color: #e0e0e0;
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
            height: 100vh;
            width: 100vw;
            overflow: hidden;
            padding: 0.75rem;
            display: flex;
            gap: 0.75rem;
            -webkit-user-select: none;
            user-select: none;
            letter-spacing: -0.025em;
        }

        /* layout utilities */
        .flex { display: flex; }
        .flex-1 { flex: 1 1 0%; }
        .flex-col { flex-direction: column; }
        .items-center { align-items: center; }
        .items-start { align-items: flex-start; }
        .items-end { align-items: flex-end; }
        .justify-between { justify-content: space-between; }
        .justify-center { justify-content: center; }
        .relative { position: relative; }
        .absolute { position: absolute; }
        .inset-0 { top: 0; right: 0; bottom: 0; left: 0; }
        .top-0 { top: 0; }
        .left-0 { left: 0; }
        .bottom-0 { bottom: 0; }
        .overflow-hidden { overflow: hidden; }
        .overflow-y-auto { overflow-y: auto; }
        .pointer-events-none { pointer-events: none; }

        /* sizing */
        .h-full { height: 100%; }
        .w-full { width: 100%; }
        .w-62 { width: 62%; }
        .w-38 { width: 38%; }
        .w-4 { width: 1rem; }
        .h-4 { height: 1rem; }
        .w-1-5 { width: 0.375rem; }
        .h-1-5 { height: 0.375rem; }
        .w-12 { width: 3rem; }
        .h-1 { height: 0.25rem; }

        /* spacing */
        .p-4 { padding: 1rem; }
        .p-5 { padding: 1.25rem; }
        .gap-3 { gap: 0.75rem; }
        .gap-2 { gap: 0.5rem; }
        .gap-1 { gap: 0.25rem; }
        .gap-4 { gap: 1rem; }
        .mt-4 { margin-top: 1rem; }
        .mt-2 { margin-top: 0.5rem; }
        .mb-1 { margin-bottom: 0.25rem; }
        .mb-2 { margin-bottom: 0.5rem; }
        .mb-3 { margin-bottom: 0.75rem; }
        .pb-2 { padding-bottom: 0.5rem; }
        .pr-2 { padding-right: 0.5rem; }
        .px-2 { padding-left: 0.5rem; padding-right: 0.5rem; }
        .py-0-5 { padding-top: 0.125rem; padding-bottom: 0.125rem; }

        /* typography */
        .text-xl { font-size: 1.25rem; line-height: 1.75rem; }
        .text-2xl { font-size: 1.5rem; line-height: 2rem; }
        .text-base { font-size: 1rem; line-height: 1.5rem; }
        .text-sm { font-size: 0.875rem; line-height: 1.25rem; }
        .text-11 { font-size: 11px; }
        .text-12 { font-size: 12px; }
        .text-10 { font-size: 10px; }
        .text-8 { font-size: 8px; }
        .text-huge { font-size: 4.8rem; }
        .text-cmd { font-size: 2.25rem; }
        .leading-none { line-height: 1; }
        .leading-relaxed { line-height: 1.625; }
        .font-bold { font-weight: 700; }
        .font-light { font-weight: 300; }
        .uppercase { text-transform: uppercase; }
        .tracking-wider { letter-spacing: 0.05em; }
        .tracking-widest { letter-spacing: 0.1em; }
        .tracking-tight { letter-spacing: -0.025em; }
        .tracking-tighter { letter-spacing: -0.05em; }
        .tracking-title { letter-spacing: 0.2em; }
        .tracking-cmd { letter-spacing: 0.1em; }
        .text-right { text-align: right; }

        /* colors */
        .text-white { color: #fff; }
        .text-nexaText { color: #e0e0e0; }
        .text-nexaMuted { color: #b5b5b5; }
        .text-nexaHighlight { color: #ffffff; }
        .text-nexaBlack { color: #000000; }

        .bg-white { background-color: #fff; }
        .bg-nexaHighlight { background-color: #ffffff; }
        .bg-nexaBorder { background-color: #2a2a2a; }
        .bg-feed { background-color: #0a0a0a; }

        .border { border-width: 1px; border-style: solid; border-color: #b5b5b5; }
        .border-b { border-bottom-width: 1px; border-bottom-style: solid; }
        .border-t { border-top-width: 1px; border-top-style: solid; }
        .border-dashed { border-style: dashed; }
        .border-nexaBorder { border-color: #2a2a2a; }
        .border-nexaMuted { border-color: #b5b5b5; }
        .border-white-20 { border-color: rgba(255,255,255,0.2); }

        .rounded-full { border-radius: 9999px; }

        .object-cover { object-fit: cover; }

        .z-10 { z-index: 10; }
        .z-20 { z-index: 20; }

        .mix-blend-difference { mix-blend-mode: difference; }

        .transition-colors { transition-property: color, background-color, border-color; }
        .duration-300 { transition-duration: 300ms; }

        /* ── original custom styles (unchanged) ── */

        .bracket-panel {
            position: relative;
            background-color: #050505;
            border: 1px solid #1a1a1a;
        }

        .bracket-panel::before, .bracket-panel::after,
        .bracket-inner::before, .bracket-inner::after {
            content: '';
            position: absolute;
            width: 8px;
            height: 8px;
            border: 1px solid #555;
            pointer-events: none;
        }

        .bracket-panel::before { top: -1px; left: -1px; border-right: none; border-bottom: none; }
        .bracket-panel::after { bottom: -1px; right: -1px; border-left: none; border-top: none; }
        .bracket-inner::before { top: -1px; right: -1px; border-left: none; border-bottom: none; }
        .bracket-inner::after { bottom: -1px; left: -1px; border-right: none; border-top: none; }

        .crosshair-h {
            position: absolute;
            top: 50%;
            left: 40%;
            right: 40%;
            height: 1px;
            background: rgba(255, 255, 255, 0.35);
            z-index: 15;
        }

        .crosshair-v {
            position: absolute;
            left: 50%;
            top: 40%;
            bottom: 40%;
            width: 1px;
            background: rgba(255, 255, 255, 0.35);
            z-index: 15;
        }

        .center-dot {
            position: absolute;
            top: 50%;
            left: 50%;
            width: 2px;
            height: 2px;
            background: #fff;
            transform: translate(-50%, -50%);
            z-index: 15;
        }

        ::-webkit-scrollbar { width: 4px; }
        ::-webkit-scrollbar-track { background: #000; }
        ::-webkit-scrollbar-thumb { background: #333; }

        .blinking { animation: blinker 1.5s linear infinite; }
        @keyframes blinker { 50% { opacity: 0; } }

        .offline-dot { background: #c1503a !important; }
    </style>
</head>
<body>

    <!-- LEFT SIDE: LIVE VIDEO FEED -->
    <div class="w-62 h-full bracket-panel relative overflow-hidden flex flex-col">
        <div class="bracket-inner absolute inset-0 pointer-events-none z-10"></div>

        <div class="absolute top-0 left-0 w-full p-4 flex justify-between items-start z-20 mix-blend-difference text-white">
            <div class="text-xl font-bold tracking-title">FIREBIRD V</div>
            <div class="flex-1"></div>
        </div>

        <div class="relative w-full h-full bg-feed">
            <img
                src="/video_feed"
                class="absolute inset-0 w-full h-full object-cover"
                alt="Live feed"
            />

            <div class="crosshair-h"></div>
            <div class="crosshair-v"></div>
            <div class="center-dot"></div>
        </div>

        <div class="absolute bottom-0 left-0 w-full p-4 flex justify-between items-end z-20 mix-blend-difference text-white text-11">
            <div class="tracking-wider">YOLOv8N-FACE VISION</div>
            <div class="flex items-center gap-2">
                <div class="w-1-5 h-1-5 bg-white blinking rounded-full" id="live-dot"></div>
                LIVE FEED
            </div>
        </div>
    </div>

    <!-- RIGHT SIDE: TELEMETRY DASHBOARD -->
    <div class="w-38 h-full flex flex-col gap-3">

        <!-- WIDGET 1: AI VISION MODULE -->
        <div class="bracket-panel flex-1 p-5 flex flex-col justify-between">
            <div class="bracket-inner absolute inset-0 pointer-events-none"></div>

            <div class="flex justify-between items-center border-b border-nexaBorder pb-2">
                <span class="text-12 text-nexaMuted uppercase tracking-widest">AI Vision State</span>
                <div class="w-4 h-4 border border-nexaMuted flex items-center justify-center text-8 text-nexaMuted">V</div>
            </div>

            <div class="flex flex-col mt-4">
                <div class="text-11 text-nexaMuted uppercase mb-1">Status</div>
                <div class="text-base text-nexaHighlight" id="lock-status">SEARCHING...</div>
            </div>

            <div class="flex justify-between items-end mt-4">
                <div>
                    <div class="text-11 text-nexaMuted uppercase mb-1">Confidence</div>
                    <div class="text-huge leading-none font-light text-nexaHighlight tracking-tighter" id="conf-val">
                        --<span class="text-2xl text-nexaMuted">%</span>
                    </div>
                </div>
                <div class="text-right">
                    <div class="text-11 text-nexaMuted uppercase mb-1">Offset Error</div>
                    <div class="text-2xl text-nexaHighlight tracking-tight" id="error-val">
                        0<span class="text-sm text-nexaMuted">px</span>
                    </div>
                </div>
            </div>
        </div>

        <!-- WIDGET 2: UPLINK KINEMATICS -->
        <div class="bracket-panel flex-1 p-5 flex flex-col justify-between">
            <div class="bracket-inner absolute inset-0 pointer-events-none"></div>

            <div class="flex justify-between items-center border-b border-nexaBorder pb-2">
                <span class="text-12 text-nexaMuted uppercase tracking-widest">Kinematics Uplink</span>
                <div class="w-4 h-4 border border-nexaMuted flex items-center justify-center text-8 text-nexaMuted">K</div>
            </div>

            <div class="flex flex-col justify-center items-center h-full mt-2">
                <div class="text-11 text-nexaMuted uppercase mb-2">Current Pi Command</div>
                <div class="text-cmd font-light tracking-cmd text-nexaHighlight" id="motor-cmd">STOP</div>

                <div class="mt-4 flex gap-1">
                    <div class="w-12 h-1 bg-nexaBorder transition-colors duration-300" id="vec-l"></div>
                    <div class="w-12 h-1 bg-nexaBorder transition-colors duration-300" id="vec-f"></div>
                    <div class="w-12 h-1 bg-nexaBorder transition-colors duration-300" id="vec-r"></div>
                </div>
            </div>
        </div>

        <!-- WIDGET 3: SYSTEM DIAGNOSTICS -->
        <div class="bracket-panel flex-1 p-5 flex flex-col justify-between">
            <div class="bracket-inner absolute inset-0 pointer-events-none"></div>

            <div class="flex justify-between items-center border-b border-nexaBorder pb-2">
                <span class="text-12 text-nexaMuted uppercase tracking-widest">System Diagnostics</span>
                <div class="w-4 h-4 border border-nexaMuted flex items-center justify-center text-8 text-nexaMuted">S</div>
            </div>

            <div class="flex flex-col gap-4 mt-4 h-full justify-center">
                <div class="flex justify-between items-center">
                    <span class="text-sm text-nexaMuted">CPU Core Temp</span>
                    <span class="text-xl text-nexaHighlight" id="temp-val">-- <span class="text-sm text-nexaMuted">C</span></span>
                </div>

                <div class="w-full border-t border-dashed border-nexaBorder"></div>

                <div class="flex justify-between items-center">
                    <span class="text-sm text-nexaMuted">Memory Load</span>
                    <span class="text-xl text-nexaHighlight" id="ram-val">-- <span class="text-sm text-nexaMuted">%</span></span>
                </div>

                <div class="w-full border-t border-dashed border-nexaBorder"></div>

                <div class="flex justify-between items-center">
                    <span class="text-sm text-nexaMuted">ATmega UART Link</span>
                    <span class="text-xl text-nexaHighlight" id="uart-val">--</span>
                </div>
            </div>
        </div>

        <!-- WIDGET 4: VOSK ACOUSTIC TERMINAL -->
        <div class="bracket-panel flex-1 p-5 flex flex-col overflow-hidden">
            <div class="bracket-inner absolute inset-0 pointer-events-none"></div>

            <div class="flex justify-between items-center border-b border-nexaBorder pb-2 mb-3">
                <span class="text-12 text-nexaMuted uppercase tracking-widest">Acoustic Terminal</span>
                <span class="text-10 text-nexaBlack bg-nexaHighlight px-2 py-0-5 border border-white-20 uppercase font-bold" id="mic-status">--</span>
            </div>

            <div class="flex-1 overflow-y-auto text-12 leading-relaxed font-mono flex flex-col gap-1 pr-2" id="terminal"></div>
        </div>

    </div>

    <script>
        const lockEl   = document.getElementById('lock-status');
        const confEl   = document.getElementById('conf-val');
        const errEl    = document.getElementById('error-val');
        const cmdEl    = document.getElementById('motor-cmd');
        const tempEl   = document.getElementById('temp-val');
        const ramEl    = document.getElementById('ram-val');
        const uartEl   = document.getElementById('uart-val');
        const micEl    = document.getElementById('mic-status');
        const termEl   = document.getElementById('terminal');
        const liveDot  = document.getElementById('live-dot');
        const vecL     = document.getElementById('vec-l');
        const vecF     = document.getElementById('vec-f');
        const vecR     = document.getElementById('vec-r');

        let lastTermCount = 0;

        function setVec(cmd) {
            const on  = "w-12 h-1 bg-white transition-colors duration-300";
            const off = "w-12 h-1 bg-nexaBorder transition-colors duration-300";
            vecL.className = (cmd === "LEFT" || cmd === "SOFT LEFT") ? on : off;
            vecF.className = (cmd === "FORWARD") ? on : off;
            vecR.className = (cmd === "RIGHT" || cmd === "SOFT RIGHT") ? on : off;
        }

        async function poll() {
            try {
                const res = await fetch('/status', { cache: 'no-store' });
                const d = await res.json();

                const locked = d.lock_status === "TARGET ACQUIRED";
                lockEl.innerText = d.lock_status;
                lockEl.className = locked ? "text-base text-nexaHighlight" : "text-base text-nexaMuted blinking";

                confEl.innerHTML = (d.confidence !== null && d.confidence !== undefined)
                    ? `${(d.confidence * 100).toFixed(1)}<span class="text-2xl text-nexaMuted">%</span>`
                    : `--<span class="text-2xl text-nexaMuted">%</span>`;

                const err = d.offset_error || 0;
                errEl.innerHTML = `${err > 0 ? '+' + err : err}<span class="text-sm text-nexaMuted">px</span>`;

                cmdEl.innerText = d.command || "STOP";
                setVec(d.command || "STOP");

                tempEl.innerHTML = (d.cpu_temp !== null && d.cpu_temp !== undefined)
                    ? `${d.cpu_temp} <span class="text-sm text-nexaMuted">C</span>` : `-- <span class="text-sm text-nexaMuted">C</span>`;
                ramEl.innerHTML = (d.ram_pct !== null && d.ram_pct !== undefined)
                    ? `${d.ram_pct} <span class="text-sm text-nexaMuted">%</span>` : `-- <span class="text-sm text-nexaMuted">%</span>`;

                uartEl.innerText = d.uart_ok ? "OK" : "OFFLINE";
                uartEl.className = d.uart_ok ? "text-xl text-nexaHighlight" : "text-xl text-nexaHighlight offline-dot";

                micEl.innerText = d.robot_active ? "ACTIVE" : "STANDBY";

                liveDot.style.background = d.uart_ok ? "#fff" : "#c1503a";

                if (d.terminal && d.terminal.length !== lastTermCount) {
                    termEl.innerHTML = d.terminal.map(line => {
                        if (line.type === "override") {
                            return `<div class="text-nexaBlack bg-nexaHighlight px-2 py-0-5" style="width:fit-content; margin-top:0.25rem; margin-bottom:0.25rem; font-weight:700; letter-spacing:0.05em;">${line.text}</div>`;
                        } else if (line.type === "heard") {
                            return `<div class="text-nexaHighlight">${line.text}</div>`;
                        }
                        return `<div class="text-nexaMuted">${line.text}</div>`;
                    }).join('');
                    termEl.scrollTop = termEl.scrollHeight;
                    lastTermCount = d.terminal.length;
                }
            } catch (e) {
                // server momentarily unreachable, retry on next tick
            }
        }

        poll();
        setInterval(poll, 400);
    </script>
</body>
</html>
"""

# ── MAIN TRACKER LOOP ────────────────────────────────────────────────────
def main():
    global latest_frame
    global robot_instance
    global robot_active

    model_path       = MODEL_FACE if Path(MODEL_FACE).exists() else MODEL_PERSON
    using_face_model = model_path == MODEL_FACE

    print(f"[MODEL] Loading {model_path}...")
    model = YOLO(model_path)
    print("[MODEL] Loaded.")

    cam = ThreadedCamera(src=CAMERA_INDEX).start()
    print("[CAM] Threaded camera stream initialized.")

    port           = find_firebird_port()
    robot_instance = RobotSerial(port, BAUD_RATE)
    lock           = FaceLock(lost_timeout=LOCK_LOST_SEC)

    frame_cx = FRAME_W // 2
    momentum_cmd = 'S'
    lost_counter = 0

    v_thread = threading.Thread(target=voice_listener_thread, daemon=True)
    v_thread.start()

    s_thread = threading.Thread(target=system_stats_thread, daemon=True)
    s_thread.start()

    print("[RUN] Active and running.")

    try:
        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            results = model(frame, imgsz=320, conf=CONF_THRESHOLD, verbose=False)

            boxes = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf   = float(box.conf[0])
                    cls_id = int(box.cls[0]) if box.cls is not None else -1
                    if not using_face_model and cls_id != 0:
                        continue
                    boxes.append((x1, y1, x2, y2, conf, cls_id))

            target_box, is_locked = lock.update(boxes)

            error = 0
            confidence = None

            if target_box is None:
                # 3-Second grace period logic: 90 frames @ 30 FPS.
                # Stop instantly if the previous command was 'F'.
                if lost_counter < 90 and momentum_cmd != 'F':
                    cmd = momentum_cmd
                    lost_counter += 1
                else:
                    cmd = "S"
            else:
                lost_counter = 0
                tx1, ty1, tx2, ty2 = target_box[:4]
                target_cx = (tx1 + tx2) // 2
                error     = target_cx - frame_cx
                cmd       = decide_command(error)
                momentum_cmd = cmd
                confidence = target_box[4]

            if robot_active:
                robot_instance.send(cmd)
            else:
                robot_instance.send("S")

            for (x1, y1, x2, y2, conf, cls_id) in boxes:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

            if target_box is not None:
                tx1, ty1, tx2, ty2 = target_box[:4]
                cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), (0, 0, 255), 2)

            with latest_frame_lock:
                latest_frame = frame.copy()

            # push live tracking telemetry to the dashboard
            update_dash(
                lock_status="TARGET ACQUIRED" if target_box is not None else "SEARCHING...",
                confidence=confidence,
                offset_error=int(error),
                command=CMD_NAME.get(cmd, "STOP"),
                robot_active=robot_active,
            )

    except KeyboardInterrupt:
        print("\n[STOP] Terminating system execution.")
    finally:
        try:
            robot_instance.close()
        except Exception:
            pass
        try:
            cam.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker_thread = threading.Thread(target=main, daemon=True)
    tracker_thread.start()
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
