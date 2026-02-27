#!/usr/bin/env python3
"""
Combined camera stream and sensor logic for Falconia.
Starts with the working stream_server code, then adds hall and ultrasonic sensor integration.
"""
from __future__ import annotations
import argparse
import logging
import threading
import time
import os
import numpy as np
import smbus2
import RPi.GPIO as GPIO

# Lazy imports for Flask & OpenCV

def _import_flask():
    from flask import Flask, Response, jsonify, request
    return Flask, Response, jsonify, request

def _import_cv2():
    import cv2
    return cv2

# FrameGrabber (from stream_server)
class FrameGrabber:
    def __init__(self, device: int = 0, width: int = 1280, height: int = 720) -> None:
        self.cv2 = _import_cv2()
        self._cap = self.cv2.VideoCapture(device)
        self._cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.frame = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=5)
        if self._cap and self._cap.isOpened():
            self._cap.release()

    def _capture_loop(self) -> None:
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

# Sensor setup
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

# Flask app factory
Flask, Response, jsonify, request = _import_flask()
app = Flask(__name__)
grabber = FrameGrabber(device=0, width=640, height=480)
grabber.start()

@app.route("/video")
def video_feed():
    return Response(grabber.generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/health")
def health():
    return jsonify({"status": "ok"})

# Sensor/image loop in background thread
def sensor_loop():
    global image_counter
    last_capture_time = 0
    last_print_time = 0
    while True:
        hall_value = read_analog(HALL_CHANNEL)
        distance = get_distance()
        current_time = time.time()
        # Print sensor values every 5 seconds
        if current_time - last_print_time >= 5:
            print(f"Hall: {hall_value}, Distance: {distance:.2f}m")
            last_print_time = current_time
        if hall_value > MAGNET_THRESHOLD:
            if current_time - last_capture_time >= 5:
                frame_bytes = grabber.get_latest_frame()
                if frame_bytes is not None:
                    np_arr = _import_cv2().imdecode(
                        np.frombuffer(frame_bytes, np.uint8), _import_cv2().IMREAD_COLOR
                    )
                    filename = f"{OUTPUT_DIR}/frame_{image_counter:04d}.jpg"
                    _import_cv2().imwrite(filename, np_arr)
                    print(f"Captured: {filename} | Distance: {distance:.2f}m")
                    image_counter += 1
                    last_capture_time = current_time
                else:
                    print("Failed to capture frame")
        time.sleep(0.1)

# Main entry point
if __name__ == "__main__":
    sensor_thread = threading.Thread(target=sensor_loop, daemon=True)
    sensor_thread.start()
    app.run(host="0.0.0.0", port=8080, use_reloader=False, threaded=True)
    grabber.stop()
    GPIO.cleanup()
