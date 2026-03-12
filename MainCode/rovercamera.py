#!/usr/bin/env python3
"""
Falconia Rover Camera — Remote Control + Live Video
====================================================

Single-process web application that combines:
  - Live MJPEG video stream (browser-viewable)
  - WASD motor control via browser keyboard input

Usage
-----
    python -m MainCode.rovercamera [--host 0.0.0.0] [--port 8080]

Then open  http://<pi-ip>:8080  in your browser.

Controls (in browser)
---------------------
    W / S  — forward / reverse
    A / D  — turn left / right
    Space  — emergency stop
"""
from __future__ import annotations

import atexit
import logging
import threading
import time
from typing import Generator, Optional

logger = logging.getLogger("falconia.rovercamera")

# ---------------------------------------------------------------------------
# Hardware availability flags
# ---------------------------------------------------------------------------
_HAS_MOTORKIT = False
_HAS_PICAMERA2 = False

try:
    import board
    from adafruit_motorkit import MotorKit
    _HAS_MOTORKIT = True
except ImportError:
    board = None  # type: ignore
    MotorKit = None  # type: ignore

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
I2C_MOTOR_ADDR   = 0x60
LEFT_MOTOR_ID    = 1
RIGHT_MOTOR_ID   = 2
MOTOR_COMMAND_TIMEOUT = 0.35


# ===================================================================
# VIDEO — MJPEG frame grabber (same approach as stream_server.py)
# ===================================================================

class FrameGrabber:
    """Capture frames in a background thread; serve as MJPEG.

    Uses the same eager-init approach as stream_server.py's FrameGrabber.
    """

    def __init__(self, device: int = 0, width: int = 1280, height: int = 720) -> None:
        if cv2 is None:
            raise ImportError("OpenCV is required. Install with: pip install opencv-python")
        self.cv2 = cv2
        self._width = width
        self._height = height
        self._device = device

        self._picam = None
        self._cap = None

        # Try Picamera2 first (Pi Camera)
        if _HAS_PICAMERA2:
            try:
                self._picam = Picamera2()
                self._picam.configure(
                    self._picam.create_video_configuration(
                        main={"size": (width, height), "format": "BGR888"}
                    )
                )
                self._picam.start()
                logger.info("FrameGrabber: using Picamera2 backend.")
            except Exception as exc:
                logger.debug("Picamera2 not available (%s), falling back to OpenCV.", exc)
                self._picam = None
                self._cap = self.cv2.VideoCapture(device)
                self._cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
                self._cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        else:
            self._cap = self.cv2.VideoCapture(device)
            self._cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.frame: Optional[bytes] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=5)
        if self._picam is not None:
            try:
                self._picam.stop()
            except Exception:
                pass
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()

    def _capture_loop(self) -> None:
        while self._running:
            raw = None
            if self._picam is not None:
                try:
                    raw = self._picam.capture_array()
                    if raw is not None and raw.shape[-1] == 3:
                        raw = self.cv2.cvtColor(raw, self.cv2.COLOR_RGB2BGR)
                except Exception:
                    time.sleep(0.05)
                    continue
            elif self._cap is not None:
                ok, raw = self._cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
            else:
                time.sleep(0.1)
                continue

            if raw is not None:
                _, buf = self.cv2.imencode(".jpg", raw, [self.cv2.IMWRITE_JPEG_QUALITY, 40])
                with self._lock:
                    self.frame = buf.tobytes()

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
        self.left_throttle = 0.0
        self.right_throttle = 0.0
        if self._left:
            self._left.throttle = None
        if self._right:
            self._right.throttle = None


# ===================================================================
# HTML — single-page control interface
# ===================================================================

CONTROL_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Falconia Rover Camera</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111;color:#e0e0e0;font-family:'Segoe UI',system-ui,sans-serif;overflow:hidden;height:100vh}
#wrap{display:flex;flex-direction:column;height:100vh}
#vid-box{flex:1;display:flex;justify-content:center;align-items:center;background:#000;position:relative;min-height:0}
#vid-box img{max-width:100%;max-height:100%;object-fit:contain}
#hud{position:absolute;top:8px;left:8px;font-size:13px;line-height:1.6;background:rgba(0,0,0,0.55);padding:6px 10px;border-radius:6px;pointer-events:none;font-family:'Courier New',monospace}
#bar{display:flex;gap:8px;padding:6px 10px;background:#1a1a1a;border-top:1px solid #333;flex-shrink:0;align-items:center;flex-wrap:wrap}
#motor-viz{display:flex;gap:4px;align-items:center;font-size:12px;font-family:monospace}
#motor-viz .mbar{width:50px;height:10px;background:#333;border-radius:2px;overflow:hidden;position:relative}
#motor-viz .mbar .fill{height:100%;position:absolute;top:0;transition:width 0.08s}
.fill.fwd{background:#4a4;left:50%}.fill.rev{background:#e44;right:50%}
#keys{margin-left:auto;font-size:11px;color:#888}
.k{display:inline-block;background:#333;border:1px solid #555;border-radius:3px;padding:1px 5px;font-family:monospace;min-width:18px;text-align:center}
.k.active{background:#4a4;color:#111;border-color:#4a4}
</style>
</head>
<body>
<div id="wrap">
  <div id="vid-box">
    <img id="vid" src="/video" alt="stream">
    <div id="hud">
      <div id="hud-motor"></div>
    </div>
  </div>
  <div id="bar">
    <div id="motor-viz">
      L<div class="mbar"><div class="fill" id="ml"></div></div>
      R<div class="mbar"><div class="fill" id="mr"></div></div>
    </div>
    <div id="keys">
      <span class="k" id="kw">W</span>
      <span class="k" id="ka">A</span>
      <span class="k" id="ks">S</span>
      <span class="k" id="kd">D</span>
      <span style="margin-left:4px" class="k" id="ksp">&#x2423;</span>
    </div>
  </div>
</div>
<script>
(function(){
  let throttle=0, steer=0;
  const keys={w:false,a:false,s:false,d:false};

  document.addEventListener('keydown',e=>{
    const k=e.key.toLowerCase();
    if(k in keys){keys[k]=true;e.preventDefault();update()}
    if(k===' '||k==='spacebar'){
      e.preventDefault();
      keys.w=false;keys.a=false;keys.s=false;keys.d=false;
      throttle=0;steer=0;sendMotor();update();
      document.getElementById('ksp').className='k active';
    }
  });
  document.addEventListener('keyup',e=>{
    const k=e.key.toLowerCase();
    if(k in keys){keys[k]=false;e.preventDefault();update()}
    if(k===' ')document.getElementById('ksp').className='k';
  });

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
    document.getElementById('kw').className='k'+(keys.w?' active':'');
    document.getElementById('ka').className='k'+(keys.a?' active':'');
    document.getElementById('ks').className='k'+(keys.s?' active':'');
    document.getElementById('kd').className='k'+(keys.d?' active':'');
  }

  let motorPending=false, motorDirty=false;
  function sendMotor(){
    if(motorPending){motorDirty=true;return;}
    motorPending=true;motorDirty=false;
    fetch('/motor',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({throttle,steer})
    }).finally(()=>{
      motorPending=false;
      if(motorDirty){motorDirty=false;sendMotor();}
    });
  }

  function poll(){
    fetch('/status').then(r=>r.json()).then(d=>{
      const lv=d.motors.left, rv=d.motors.right;
      setBar(document.getElementById('ml'),lv);
      setBar(document.getElementById('mr'),rv);
      document.getElementById('hud-motor').textContent=
        'Motors  L:'+fmt(lv)+' R:'+fmt(rv);
    }).catch(()=>{});
  }

  function setBar(el,v){
    const pct=Math.abs(v)*50;
    el.style.width=pct+'%';
    el.className='fill '+(v>=0?'fwd':'rev');
  }
  function fmt(v){return (v>=0?'+':'')+v.toFixed(2)}

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

    grabber = FrameGrabber()
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
                _stop()
            time.sleep(0.05)

    threading.Thread(target=_motor_watchdog, daemon=True).start()

    def _shutdown():
        motors.release()
        grabber.stop()
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
    p = argparse.ArgumentParser(description="Falconia Rover Camera")
    p.add_argument("--host", default="0.0.0.0", help="Bind address (default 0.0.0.0)")
    p.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080)")
    args = p.parse_args()

    print(f"\n  Falconia Rover Camera")
    print(f"  ─────────────────────")
    print(f"  Open in browser:  http://<pi-ip>:{args.port}")
    print(f"  Motors:  {'ready' if _HAS_MOTORKIT else 'not available'}")
    print(f"  Camera:  {'Pi Camera' if _HAS_PICAMERA2 else ('USB/OpenCV' if cv2 else 'none')}")
    print(f"\n  Controls: WASD = drive, Space = stop\n")
    print("  Initializing services...\n")

    app = create_app()
    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
