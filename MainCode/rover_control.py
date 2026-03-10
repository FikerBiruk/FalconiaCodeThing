#!/usr/bin/env python3
"""
Falconia Rover — Unified Control Interface
==========================================

Single-process web application that combines:
  - Live MJPEG video stream (browser-viewable)
  - WASD motor control via browser keyboard input
  - Slope profiler (ultrasonic HC-SR04)
  - Hall sensor (PCF8591 ADC)

All controllable from one browser tab. Sensors can be toggled on/off
to save CPU/memory. Designed to run lean over SSH on a Raspberry Pi.

Usage
-----
    python -m MainCode.rover_control [--host 0.0.0.0] [--port 8080]

Then open  http://<pi-ip>:8080  in your browser.

Controls (in browser)
---------------------
    W / S  — forward / reverse
    A / D  — turn left / right
    Space  — emergency stop
    1      — toggle slope profiler on/off
    2      — toggle hall sensor on/off
"""
from __future__ import annotations

import atexit
import logging
import math
import threading
import time
from typing import Generator, Optional

logger = logging.getLogger("falconia.rover")

# ---------------------------------------------------------------------------
# Hardware availability flags (set at import time)
# ---------------------------------------------------------------------------
_HAS_GPIO = False
_HAS_MOTORKIT = False
_HAS_SMBUS = False
_HAS_PICAMERA2 = False

try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    GPIO = None

try:
    import board
    from adafruit_motorkit import MotorKit
    _HAS_MOTORKIT = True
except ImportError:
    board = None  # type: ignore
    MotorKit = None  # type: ignore

try:
    import smbus2
    _HAS_SMBUS = True
except ImportError:
    smbus2 = None  # type: ignore

try:
    from picamera2 import Picamera2
    _HAS_PICAMERA2 = True
except ImportError:
    Picamera2 = None  # type: ignore

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore

from flask import Flask, Response, jsonify, request

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# Motors
I2C_MOTOR_ADDR   = 0x60
LEFT_MOTOR_ID    = 1
RIGHT_MOTOR_ID   = 2

# Hall sensor (PCF8591 ADC)
PCF8591_ADDR     = 0x48
HALL_CHANNEL     = 0

# Ultrasonic (HC-SR04) — used by slope profiler
TRIG_PIN         = 23
ECHO_PIN         = 24
MOUNTING_ANGLE   = 45.0   # sensor tilt in degrees

# Sensor polling
SENSOR_INTERVAL  = 0.15   # seconds between sensor reads
MOTOR_COMMAND_TIMEOUT = 0.35  # auto-stop if no command arrives (network jitter safety)


# ===================================================================
# VIDEO — lightweight MJPEG frame grabber
# ===================================================================

class FrameGrabber:
    """Capture frames in a background thread; serve as MJPEG."""

    def __init__(self, width: int = 640, height: int = 480) -> None:
        self._width = width
        self._height = height
        self._picam = None
        self._cap = None
        self._backend_ready = False
        self.frame: Optional[bytes] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def _init_backend(self) -> None:
        """Initialize camera backend from background thread to avoid blocking startup."""
        if self._backend_ready:
            return

        # Try Pi Camera first, then USB
        if _HAS_PICAMERA2:
            try:
                self._picam = Picamera2()
                self._picam.configure(
                    self._picam.create_video_configuration(
                        main={"size": (self._width, self._height), "format": "BGR888"}
                    )
                )
                self._picam.start()
                self._backend_ready = True
                logger.info("FrameGrabber: Picamera2 backend")
                return
            except Exception as e:
                logger.warning("Picamera2 failed (%s), trying OpenCV", e)
                self._picam = None

        if cv2 is not None:
            try:
                self._cap = cv2.VideoCapture(0)
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                self._backend_ready = True
                logger.info("FrameGrabber: OpenCV backend")
            except Exception as e:
                logger.warning("OpenCV camera init failed: %s", e)
                self._cap = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._picam:
            try:
                self._picam.stop()
            except Exception:
                pass
        if self._cap and self._cap.isOpened():
            self._cap.release()

    def _loop(self) -> None:
        encode_param = [cv2.IMWRITE_JPEG_QUALITY, 40] if cv2 else []
        while self._running:
            if not self._backend_ready:
                self._init_backend()
                if not self._backend_ready:
                    time.sleep(0.5)
                    continue

            raw = None
            if self._picam:
                try:
                    raw = self._picam.capture_array()
                except Exception:
                    time.sleep(0.05)
                    continue
            elif self._cap:
                ok, raw = self._cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
            else:
                time.sleep(0.2)
                continue

            if raw is not None and cv2 is not None:
                _, buf = cv2.imencode(".jpg", raw, encode_param)
                with self._lock:
                    self.frame = buf.tobytes()
            time.sleep(0.033)  # ~30 fps cap

    def generate(self) -> Generator[bytes, None, None]:
        while self._running:
            with self._lock:
                f = self.frame
            if f is None:
                time.sleep(0.05)
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + f + b"\r\n"
            )
            time.sleep(0.033)


# ===================================================================
# MOTORS — thin wrapper around MotorKit
# ===================================================================

class MotorController:
    """Differential-drive motor controller."""

    def __init__(self) -> None:
        self._kit = None
        self._left = None
        self._right = None
        self.left_throttle = 0.0
        self.right_throttle = 0.0

        if _HAS_MOTORKIT:
            try:
                self._kit = MotorKit(i2c=board.I2C(), address=I2C_MOTOR_ADDR)
                self._left = self._kit.motor1 if LEFT_MOTOR_ID == 1 else self._kit.motor2
                self._right = self._kit.motor2 if RIGHT_MOTOR_ID == 2 else self._kit.motor1
                logger.info("MotorController: MotorKit ready")
            except Exception as e:
                logger.warning("MotorKit init failed: %s", e)

    @property
    def available(self) -> bool:
        return self._left is not None

    def drive(self, throttle: float, steer: float) -> None:
        """Set differential drive from throttle [-1,1] + steer [-1,1]."""
        left = max(-1.0, min(1.0, throttle + steer))
        right = max(-1.0, min(1.0, throttle - steer))
        self.left_throttle = left
        self.right_throttle = right
        if self._left:
            self._left.throttle = left
        if self._right:
            self._right.throttle = right

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def release(self) -> None:
        """Set throttle to None (coast)."""
        self.left_throttle = 0.0
        self.right_throttle = 0.0
        if self._left:
            self._left.throttle = None
        if self._right:
            self._right.throttle = None


# ===================================================================
# HALL SENSOR — PCF8591 ADC reader
# ===================================================================

class HallSensor:
    """Read hall sensor via PCF8591 8-bit ADC over I2C.

    Value interpretation:
        ~150         → no magnet / baseline
        150-160      → ferromagnetic material nearby
        >160         → magnetic material detected
    """

    def __init__(self, address: int = PCF8591_ADDR, channel: int = HALL_CHANNEL):
        self._addr = address
        self._channel = channel
        self._bus = None

    def _ensure_bus(self) -> bool:
        if self._bus is not None:
            return True
        if not _HAS_SMBUS:
            return False
        try:
            self._bus = smbus2.SMBus(1)
            logger.info("HallSensor: ready on 0x%02X ch%d", self._addr, self._channel)
            return True
        except Exception as e:
            logger.warning("HallSensor init failed: %s", e)
            return False

    @property
    def available(self) -> bool:
        return self._bus is not None or _HAS_SMBUS

    def read(self) -> Optional[int]:
        """Return 8-bit ADC value (0-255), or None on failure."""
        if not self._ensure_bus():
            return None
        try:
            self._bus.write_byte(self._addr, 0x40 | (self._channel & 0x03))
            self._bus.read_byte(self._addr)  # discard first (stale)
            return self._bus.read_byte(self._addr)
        except Exception:
            return None

    def classify(self, value: int) -> str:
        """Classify a hall reading."""
        if value < 150:
            return "no_magnet"
        elif value <= 160:
            return "ferromagnetic"
        else:
            return "magnetic"

    def cleanup(self) -> None:
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass


# ===================================================================
# SLOPE PROFILER — ultrasonic distance + sliding-window regression
# ===================================================================

class UltrasonicReader:
    """HC-SR04 distance reader via GPIO."""

    SPEED_OF_SOUND = 34300.0  # cm/s at ~20C
    TIMEOUT = 0.04            # 40 ms

    def __init__(self, trig: int = TRIG_PIN, echo: int = ECHO_PIN):
        self._trig = trig
        self._echo = echo
        self._ready = False
        if _HAS_GPIO:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(trig, GPIO.OUT)
                GPIO.setup(echo, GPIO.IN)
                GPIO.output(trig, GPIO.LOW)
                time.sleep(0.05)
                self._ready = True
                logger.info("UltrasonicReader: GPIO ready (trig=%d echo=%d)", trig, echo)
            except Exception as e:
                logger.warning("Ultrasonic init failed: %s", e)

    @property
    def available(self) -> bool:
        return self._ready

    def read_cm(self) -> Optional[float]:
        if not self._ready:
            return None
        try:
            GPIO.output(self._trig, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self._trig, GPIO.LOW)

            deadline = time.monotonic() + self.TIMEOUT
            start = time.monotonic()
            while GPIO.input(self._echo) == GPIO.LOW:
                start = time.monotonic()
                if start > deadline:
                    return None

            end = start
            while GPIO.input(self._echo) == GPIO.HIGH:
                end = time.monotonic()
                if end > deadline:
                    return None

            dist = (end - start) * self.SPEED_OF_SOUND / 2.0
            if dist < 2.0 or dist > 400.0:
                return None
            return round(dist, 2)
        except Exception:
            return None

    def cleanup(self) -> None:
        if _HAS_GPIO:
            try:
                GPIO.cleanup()
            except Exception:
                pass


class SlopeEstimator:
    """Sliding-window linear regression on ultrasonic distance samples."""

    def __init__(self, window: int = 15, alpha: float = 0.3,
                 mount_angle: float = MOUNTING_ANGLE):
        self._window = window
        self._alpha = alpha
        self._mount = mount_angle
        self._times: list[float] = []
        self._dists: list[float] = []
        self._smooth = 0.0
        self._init = False
        self._lock = threading.Lock()

    def add(self, dist_cm: float, ts: Optional[float] = None) -> dict:
        with self._lock:
            return self._add_locked(dist_cm, ts)

    def _add_locked(self, dist_cm: float, ts: Optional[float] = None) -> dict:
        t = ts or time.monotonic()
        self._times.append(t)
        self._dists.append(dist_cm)
        # trim to window
        if len(self._times) > self._window:
            self._times = self._times[-self._window:]
            self._dists = self._dists[-self._window:]

        n = len(self._times)
        if n < 2:
            return {"slope_deg": 0.0, "smoothed_deg": 0.0,
                    "distance_cm": dist_cm, "condition": "flat"}

        # OLS regression
        t0 = self._times[0]
        xs = [t - t0 for t in self._times]
        ys = self._dists
        sx = sum(xs)
        sy = sum(ys)
        sxy = sum(x * y for x, y in zip(xs, ys))
        sx2 = sum(x * x for x in xs)
        denom = n * sx2 - sx * sx
        if abs(denom) < 1e-12:
            slope_cms = 0.0
        else:
            slope_cms = (n * sxy - sx * sy) / denom

        raw_deg = math.degrees(math.atan(slope_cms))
        slope_deg = raw_deg - self._mount

        if not self._init:
            self._smooth = slope_deg
            self._init = True
        else:
            self._smooth = self._alpha * slope_deg + (1 - self._alpha) * self._smooth

        a = abs(slope_deg)
        if a >= 60:
            cond = "vertical"
        elif a >= 15:
            cond = "steep_slope"
        elif a >= 5:
            cond = "gentle_slope"
        else:
            cond = "flat"

        return {
            "slope_deg": round(slope_deg, 1),
            "smoothed_deg": round(self._smooth, 1),
            "distance_cm": round(dist_cm, 1),
            "condition": cond,
        }

    def reset(self) -> None:
        with self._lock:
            self._times.clear()
            self._dists.clear()
            self._smooth = 0.0
            self._init = False


# ===================================================================
# SENSOR MANAGER — runs enabled sensors in a background thread
# ===================================================================

class SensorManager:
    """Manages sensor reading in a single background thread.

    Sensors can be toggled on/off at runtime. Latest readings are
    stored and returned as JSON-friendly dicts.
    """

    def __init__(self) -> None:
        self.hall = HallSensor()
        self.ultrasonic = UltrasonicReader()
        self.slope = SlopeEstimator()

        self.hall_enabled = False
        self.slope_enabled = False

        self._hall_data: dict = {}
        self._slope_data: dict = {}
        self._lock = threading.Lock()
        self._running = False

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self) -> None:
        self._running = False
        self.hall.cleanup()
        self.ultrasonic.cleanup()

    def _loop(self) -> None:
        while self._running:
            # Hall sensor
            if self.hall_enabled and self.hall.available:
                val = self.hall.read()
                if val is not None:
                    with self._lock:
                        self._hall_data = {
                            "value": val,
                            "classification": self.hall.classify(val),
                            "voltage": round(val * 3.3 / 255, 2),
                        }

            # Slope profiler (ultrasonic)
            if self.slope_enabled and self.ultrasonic.available:
                dist = self.ultrasonic.read_cm()
                if dist is not None:
                    data = self.slope.add(dist)
                    with self._lock:
                        self._slope_data = data

            time.sleep(SENSOR_INTERVAL)

    def get_readings(self) -> dict:
        with self._lock:
            return {
                "hall": {
                    "enabled": self.hall_enabled,
                    "available": self.hall.available,
                    "data": dict(self._hall_data) if self._hall_data else None,
                },
                "slope": {
                    "enabled": self.slope_enabled,
                    "available": self.ultrasonic.available,
                    "data": dict(self._slope_data) if self._slope_data else None,
                },
            }


# ===================================================================
# HTML — single-page control interface
# ===================================================================

CONTROL_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Falconia Rover</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111;color:#e0e0e0;font-family:'Segoe UI',system-ui,sans-serif;overflow:hidden;height:100vh}
#wrap{display:flex;flex-direction:column;height:100vh}
#vid-box{flex:1;display:flex;justify-content:center;align-items:center;background:#000;position:relative;min-height:0}
#vid-box img{max-width:100%;max-height:100%;object-fit:contain}
#hud{position:absolute;top:8px;left:8px;font-size:13px;line-height:1.6;background:rgba(0,0,0,0.55);padding:6px 10px;border-radius:6px;pointer-events:none;font-family:'Courier New',monospace}
#bar{display:flex;gap:8px;padding:6px 10px;background:#1a1a1a;border-top:1px solid #333;flex-shrink:0;align-items:center;flex-wrap:wrap}
#bar .sensor-block{display:flex;align-items:center;gap:6px;padding:3px 8px;border-radius:4px;font-size:12px;background:#222;border:1px solid #333}
#bar .sensor-block.on{border-color:#4a4}
#bar .sensor-block.off{opacity:0.5}
.tag{font-weight:bold;font-size:11px;text-transform:uppercase;letter-spacing:0.5px}
.val{color:#7bf}
#keys{margin-left:auto;font-size:11px;color:#888}
.k{display:inline-block;background:#333;border:1px solid #555;border-radius:3px;padding:1px 5px;font-family:monospace;min-width:18px;text-align:center}
.k.active{background:#4a4;color:#111;border-color:#4a4}
#motor-viz{display:flex;gap:4px;align-items:center;font-size:12px;font-family:monospace}
#motor-viz .mbar{width:50px;height:10px;background:#333;border-radius:2px;overflow:hidden;position:relative}
#motor-viz .mbar .fill{height:100%;position:absolute;top:0;transition:width 0.08s}
.fill.fwd{background:#4a4;left:50%}.fill.rev{background:#e44;right:50%}
/* condition colors */
.cond-flat{color:#4a4}.cond-gentle_slope{color:#ea4}.cond-steep_slope{color:#e84}.cond-vertical{color:#e44}
.mag-no_magnet{color:#888}.mag-ferromagnetic{color:#ea4}.mag-magnetic{color:#e44}
</style>
</head>
<body>
<div id="wrap">
  <div id="vid-box">
    <img id="vid" src="/video" alt="stream">
    <div id="hud">
      <div id="hud-motor"></div>
      <div id="hud-slope"></div>
      <div id="hud-hall"></div>
    </div>
  </div>
  <div id="bar">
    <div id="blk-slope" class="sensor-block off">
      <span class="tag">1 Slope</span>
      <span class="val" id="v-slope">OFF</span>
    </div>
    <div id="blk-hall" class="sensor-block off">
      <span class="tag">2 Hall</span>
      <span class="val" id="v-hall">OFF</span>
    </div>
    <div id="motor-viz">
      L<div class="mbar"><div class="fill" id="ml"></div></div>
      R<div class="mbar"><div class="fill" id="mr"></div></div>
    </div>
    <div id="keys">
      <span class="k" id="kw">W</span>
      <span class="k" id="ka">A</span>
      <span class="k" id="ks">S</span>
      <span class="k" id="kd">D</span>
      <span style="margin-left:4px" class="k" id="ksp">␣</span>
    </div>
  </div>
</div>
<script>
(function(){
  // Motor state
  let throttle=0, steer=0;
  const keys={w:false,a:false,s:false,d:false};
  let slopeOn=false, hallOn=false;

  // Track toggle keys to prevent repeat-firing
  const toggleHeld={1:false,2:false};

  // Key handling
  document.addEventListener('keydown',e=>{
    const k=e.key.toLowerCase();
    if(k in keys){keys[k]=true;e.preventDefault();update()}
    if(k===' '||k==='spacebar'){
      e.preventDefault();
      keys.w=false;keys.a=false;keys.s=false;keys.d=false;
      throttle=0;steer=0;sendMotor();update();
      document.getElementById('ksp').className='k active';
    }
    if(k==='1'&&!toggleHeld[1]){toggleHeld[1]=true;toggleSensor('slope')}
    if(k==='2'&&!toggleHeld[2]){toggleHeld[2]=true;toggleSensor('hall')}
  });
  document.addEventListener('keyup',e=>{
    const k=e.key.toLowerCase();
    if(k in keys){keys[k]=false;e.preventDefault();update()}
    if(k===' ')document.getElementById('ksp').className='k';
    if(k==='1')toggleHeld[1]=false;
    if(k==='2')toggleHeld[2]=false;
  });

  // Safety: stop motors if browser loses focus
  window.addEventListener('blur',()=>{
    keys.w=false;keys.a=false;keys.s=false;keys.d=false;
    throttle=0;steer=0;sendMotor();update();
  });
  document.addEventListener('visibilitychange',()=>{
    if(document.hidden){
      keys.w=false;keys.a=false;keys.s=false;keys.d=false;
      throttle=0;steer=0;sendMotor();update();
    }
  });

  function update(){
    throttle=keys.w?1:(keys.s?-1:0);
    steer=keys.d?1:(keys.a?-1:0);
    sendMotor();
    // Key indicators
    document.getElementById('kw').className='k'+(keys.w?' active':'');
    document.getElementById('ka').className='k'+(keys.a?' active':'');
    document.getElementById('ks').className='k'+(keys.s?' active':'');
    document.getElementById('kd').className='k'+(keys.d?' active':'');
  }

  // Send motor command (fire-and-forget, throttled)
  let motorPending=false;
  let motorDirty=false;
  function sendMotor(){
    if(motorPending){motorDirty=true;return;}
    motorPending=true;
    motorDirty=false;
    fetch('/motor',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({throttle,steer})
    }).finally(()=>{
      motorPending=false;
      if(motorDirty){motorDirty=false;sendMotor();}
    });
  }

  // Toggle sensors
  function toggleSensor(name){
    fetch('/sensor/toggle',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({sensor:name})
    }).then(r=>r.json()).then(d=>{
      if(name==='slope')slopeOn=d.enabled;
      if(name==='hall')hallOn=d.enabled;
      updateSensorUI();
    });
  }

  function updateSensorUI(){
    const bs=document.getElementById('blk-slope');
    bs.className='sensor-block '+(slopeOn?'on':'off');
    const bh=document.getElementById('blk-hall');
    bh.className='sensor-block '+(hallOn?'on':'off');
  }

  // Poll sensor data + motor state
  function poll(){
    fetch('/status').then(r=>r.json()).then(d=>{
      // Motors
      const lv=d.motors.left, rv=d.motors.right;
      const ml=document.getElementById('ml');
      const mr=document.getElementById('mr');
      setBar(ml,lv);setBar(mr,rv);
      document.getElementById('hud-motor').textContent=
        'Motors  L:'+fmt(lv)+' R:'+fmt(rv);

      // Slope
      slopeOn=d.sensors.slope.enabled;
      const sd=d.sensors.slope.data;
      const vs=document.getElementById('v-slope');
      const hs=document.getElementById('hud-slope');
      if(!slopeOn){vs.textContent='OFF';hs.textContent=''}
      else if(sd){
        vs.innerHTML=sd.slope_deg+'° <span class="cond-'+sd.condition+'">'+sd.condition+'</span>';
        hs.innerHTML='Slope   '+sd.slope_deg+'° (smooth '+sd.smoothed_deg+'°) dist='+sd.distance_cm+'cm <span class="cond-'+sd.condition+'">'+sd.condition+'</span>';
      }else{vs.textContent='waiting…';hs.textContent='Slope   waiting…'}

      // Hall
      hallOn=d.sensors.hall.enabled;
      const hd=d.sensors.hall.data;
      const vh=document.getElementById('v-hall');
      const hh=document.getElementById('hud-hall');
      if(!hallOn){vh.textContent='OFF';hh.textContent=''}
      else if(hd){
        vh.innerHTML=hd.value+' <span class="mag-'+hd.classification+'">'+hd.classification+'</span>';
        hh.innerHTML='Hall    val='+hd.value+' ('+hd.voltage+'V) <span class="mag-'+hd.classification+'">'+hd.classification+'</span>';
      }else{vh.textContent='waiting…';hh.textContent='Hall    waiting…'}

      updateSensorUI();
    }).catch(()=>{});
  }

  function setBar(el,v){
    const pct=Math.abs(v)*50;
    el.style.width=pct+'%';
    el.className='fill '+(v>=0?'fwd':'rev');
  }
  function fmt(v){return (v>=0?'+':'')+v.toFixed(2)}

    // Heartbeat while moving: helps absorb packet loss/jitter on SSH links.
    setInterval(()=>{
        if(throttle!==0 || steer!==0)sendMotor();
    },120);

  setInterval(poll,250);
})();
</script>
</body>
</html>"""


# ===================================================================
# FLASK APP
# ===================================================================

def create_app() -> Flask:
    app = Flask(__name__)
    app.logger.setLevel(logging.WARNING)

    # --- Components ---
    grabber = FrameGrabber(width=640, height=480)
    grabber.start()

    motors = MotorController()
    motor_lock = threading.Lock()
    last_motor_cmd_ts = time.monotonic()

    def _drive(t: float, s: float) -> None:
        with motor_lock:
            motors.drive(t, s)

    def _stop() -> None:
        with motor_lock:
            motors.stop()

    def _motor_watchdog() -> None:
        nonlocal last_motor_cmd_ts
        while True:
            elapsed = time.monotonic() - last_motor_cmd_ts
            if elapsed > MOTOR_COMMAND_TIMEOUT:
                # Auto-stop prevents stale forward commands on dropped keyup packets.
                _stop()
            time.sleep(0.05)

    threading.Thread(target=_motor_watchdog, daemon=True).start()

    sensors = SensorManager()
    sensors.start()

    def _shutdown():
        motors.release()
        grabber.stop()
        sensors.stop()
    atexit.register(_shutdown)

    # --- Routes ---

    @app.route("/")
    def index():
        return Response(CONTROL_PAGE, content_type="text/html")

    @app.route("/video")
    def video():
        return Response(grabber.generate(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/motor", methods=["POST"])
    def motor():
        nonlocal last_motor_cmd_ts
        d = request.get_json(silent=True) or {}
        try:
            t = max(-1.0, min(1.0, float(d.get("throttle", 0))))
            s = max(-1.0, min(1.0, float(d.get("steer", 0))))
        except (ValueError, TypeError):
            return "", 400
        _drive(t, s)
        last_motor_cmd_ts = time.monotonic()
        return "", 204

    @app.route("/sensor/toggle", methods=["POST"])
    def sensor_toggle():
        d = request.get_json(silent=True) or {}
        name = d.get("sensor", "")
        enabled = False
        if name == "slope":
            sensors.slope_enabled = not sensors.slope_enabled
            if not sensors.slope_enabled:
                sensors.slope.reset()
            enabled = sensors.slope_enabled
        elif name == "hall":
            sensors.hall_enabled = not sensors.hall_enabled
            enabled = sensors.hall_enabled
        return jsonify({"sensor": name, "enabled": enabled})

    @app.route("/status")
    def status():
        with motor_lock:
            left = round(motors.left_throttle, 2)
            right = round(motors.right_throttle, 2)
        return jsonify({
            "motors": {
                "left": left,
                "right": right,
                "available": motors.available,
            },
            "sensors": sensors.get_readings(),
        })

    @app.route("/health")
    def health():
        return jsonify({"status": "ok"})

    return app


# ===================================================================
# CLI
# ===================================================================

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    logging.getLogger("picamera2").setLevel(logging.WARNING)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)

    import argparse
    p = argparse.ArgumentParser(description="Falconia Rover Control")
    p.add_argument("--host", default="0.0.0.0", help="Bind address (default 0.0.0.0)")
    p.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080)")
    args = p.parse_args()

    print(f"\n  Falconia Rover Control")
    print(f"  ─────────────────────")
    print(f"  Open in browser:  http://<pi-ip>:{args.port}")
    print(f"  Motors:    {'ready' if _HAS_MOTORKIT else 'not available'}")
    print(f"  Camera:    {'Pi Camera' if _HAS_PICAMERA2 else ('USB/OpenCV' if cv2 else 'none')}")
    print(f"  Hall:      {'ready' if _HAS_SMBUS else 'not available'}")
    print(f"  Ultrasonic: {'ready' if _HAS_GPIO else 'not available'}")
    print(f"\n  Controls: WASD=drive, Space=stop, 1=slope, 2=hall\n")
    print("  Initializing services...\n")

    app = create_app()

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
