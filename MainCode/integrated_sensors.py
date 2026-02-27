
import time
import threading
import smbus2
import RPi.GPIO as GPIO
import os
from flask import Flask, Response
import cv2

# Camera stream setup
class FrameGrabber:
    def __init__(self, device=0, width=640, height=480):
        self.cv2 = cv2
        self._cap = self.cv2.VideoCapture(device)
        self._cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
        if self._cap and self._cap.isOpened():
            self._cap.release()

    def _capture_loop(self):
        while self._running:
            ok, raw = self._cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            _, buf = self.cv2.imencode(".jpg", raw, [self.cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._lock:
                self.frame = buf.tobytes()
            time.sleep(0.033)

    def get_latest_frame(self):
        with self._lock:
            return self.frame

    def get_latest_raw(self):
        with self._lock:
            if self._cap:
                ok, raw = self._cap.read()
                if ok:
                    return raw
            return None

    def generate_mjpeg(self):
        while self._running:
            frame = self.get_latest_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            time.sleep(0.033)

# Flask app for MJPEG stream
app = Flask(__name__)
grabber = FrameGrabber(device=0, width=640, height=480)
grabber.start()

@app.route("/video")
def video_feed():
    return Response(grabber.generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

def run_flask():
    app.run(host="0.0.0.0", port=8080, threaded=True)

# Hall sensor setup
PCF8591_ADDRESS = 0x48
HALL_CHANNEL = 0
MAGNET_THRESHOLD = 100
bus = smbus2.SMBus(1)

def read_analog(channel):
    control_byte = channel
    bus.write_byte(PCF8591_ADDRESS, control_byte)
    bus.read_byte(PCF8591_ADDRESS)
    value = bus.read_byte(PCF8591_ADDRESS)
    return value

# Ultrasonic setup
TRIG = 23
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start = time.time()
    end = time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    while GPIO.input(ECHO) == 1:
        end = time.time()
    duration = end - start
    distance = duration * 17150
    return distance / 100.0

OUTPUT_DIR = "captures"
image_counter = 0
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

last_capture_time = 0

# Start Flask app in background thread
flask_thread = threading.Thread(target=run_flask, daemon=True)
flask_thread.start()

try:
    while True:
        hall_value = read_analog(HALL_CHANNEL)
        distance = get_distance()
        print(f"Hall: {hall_value}, Distance: {distance:.2f}m")

        if hall_value > MAGNET_THRESHOLD:
            current_time = time.time()
            if current_time - last_capture_time >= 5:
                # Use latest raw frame from grabber
                raw_frame = grabber.get_latest_raw()
                if raw_frame is not None:
                    filename = f"{OUTPUT_DIR}/frame_{image_counter:04d}.jpg"
                    cv2.imwrite(filename, raw_frame)
                    print(f"Captured: {filename} | Distance: {distance:.2f}m")
                    image_counter += 1
                    last_capture_time = current_time
                else:
                    print("Failed to capture frame")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    grabber.stop()
    GPIO.cleanup()
