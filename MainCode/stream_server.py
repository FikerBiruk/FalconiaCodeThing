#!/usr/bin/env python3
"""Flask endpoint that proxies the Falconia camera stream.

Provides:
    GET /video          – MJPEG stream (browser-compatible preview)
    POST /stream/start  – Start the GStreamer RTP/UDP stream
    POST /stream/stop   – Stop the GStreamer RTP/UDP stream
    GET /stream/status  – JSON status of the active stream
    GET /health         – Simple liveness probe

The ``/video`` MJPEG endpoint is independent of the GStreamer pipeline.
It captures frames locally (via Picamera2 or OpenCV) and serves them as a
multipart JPEG stream so you can view video directly in a browser.

The ``/stream/*`` endpoints control the RTP/UDP H.264 pipeline that streams
to a remote client (which uses the corresponding GStreamer receiver).

Usage
-----
    python -m MainCode.stream_server --ip 192.168.1.100 --port 5000

Requires
--------
    pip install flask opencv-python
"""
from __future__ import annotations

import argparse
import logging
import threading
import time
from typing import Generator, Optional

logger = logging.getLogger("falconia.stream_server")

# ---------------------------------------------------------------------------
# Lazy imports – Flask & OpenCV may not be installed everywhere.
# ---------------------------------------------------------------------------

def _import_flask():
    try:
        from flask import Flask, Response, jsonify, request
        return Flask, Response, jsonify, request
    except ImportError:
        raise ImportError(
            "Flask is required for the stream server. "
            "Install with:  pip install flask"
        )


def _import_cv2():
    try:
        import cv2
        return cv2
    except ImportError:
        raise ImportError(
            "OpenCV is required for the MJPEG preview. "
            "Install with:  pip install opencv-python"
        )


# ---------------------------------------------------------------------------
# Frame generator for MJPEG
# ---------------------------------------------------------------------------

class FrameGrabber:
    """Continuously capture frames in a background thread.

    Tries Picamera2 first (for Pi Camera), falls back to OpenCV VideoCapture
    (for USB webcams).  The latest JPEG-encoded frame is always available
    via ``.frame``.
    """

    def __init__(self, device: int = 0, width: int = 1280, height: int = 720) -> None:
        self.cv2 = _import_cv2()
        self._width = width
        self._height = height
        self._device = device

        self._picam = None
        self._cap = None

        # Try Picamera2 first (Pi Camera)
        try:
            from picamera2 import Picamera2
            self._picam = Picamera2()
            self._picam.configure(
                self._picam.create_video_configuration(
                    main={"size": (width, height), "format": "RGB888"}
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

            _, buf = self.cv2.imencode(".jpg", raw, [self.cv2.IMWRITE_JPEG_QUALITY, 70])
            with self._lock:
                self.frame = buf.tobytes()

    def generate_mjpeg(self) -> Generator[bytes, None, None]:
        """Yield multipart JPEG frames suitable for an HTTP response."""
        while self._running:
            with self._lock:
                frame = self.frame
            if frame is None:
                time.sleep(0.05)
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
            # ~30 fps cap
            time.sleep(0.033)


# ---------------------------------------------------------------------------
# Flask app factory
# ---------------------------------------------------------------------------

def create_app(
    target_ip: str = "127.0.0.1",
    target_port: int = 5000,
    video_device: int = 0,
) -> "Flask":
    """Create and return the Flask application.

    Parameters
    ----------
    target_ip : str
        Default target IP for the RTP/UDP stream.
    target_port : int
        Default target UDP port.
    video_device : int
        OpenCV device index for MJPEG preview.
    """
    from MainCode.camera_manager import CameraManager, CameraType

    Flask, Response, jsonify, request = _import_flask()

    app = Flask(__name__)
    cam_manager = CameraManager(target_ip=target_ip, target_port=target_port)
    grabber = FrameGrabber(device=video_device)
    grabber.start()

    # TODO: Plug in Falconia authentication / access control here.

    @app.route("/health")
    def health():
        """Liveness probe."""
        return jsonify({"status": "ok"})

    @app.route("/video")
    def video_feed():
        """Browser-viewable MJPEG stream."""
        return Response(
            grabber.generate_mjpeg(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/stream/start", methods=["POST"])
    def stream_start():
        """Start the GStreamer RTP/UDP pipeline.

        Optional JSON body::

            {"ip": "192.168.1.50", "port": 5000}
        """
        data = request.get_json(silent=True) or {}
        cam_manager.target_ip = data.get("ip", cam_manager.target_ip)
        cam_manager.target_port = int(data.get("port", cam_manager.target_port))

        try:
            cam_manager.start_stream()
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 500
        return jsonify({"status": "streaming", "target": f"{cam_manager.target_ip}:{cam_manager.target_port}"})

    @app.route("/stream/stop", methods=["POST"])
    def stream_stop():
        """Stop the GStreamer RTP/UDP pipeline."""
        cam_manager.stop_stream()
        return jsonify({"status": "stopped"})

    @app.route("/stream/status")
    def stream_status():
        """Return current streaming state."""
        return jsonify({
            "streaming": cam_manager.is_streaming,
            "camera_type": cam_manager.camera_type.value,
            "target_ip": cam_manager.target_ip,
            "target_port": cam_manager.target_port,
        })

    @app.teardown_appcontext
    def _cleanup(_exc: object = None) -> None:
        cam_manager.stop_stream()
        grabber.stop()

    return app


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="Falconia streaming server")
    parser.add_argument("--ip", default="127.0.0.1", help="RTP target IP")
    parser.add_argument("--port", type=int, default=5000, help="RTP target UDP port")
    parser.add_argument("--flask-host", default="0.0.0.0", help="Flask bind address")
    parser.add_argument("--flask-port", type=int, default=8080, help="Flask HTTP port")
    parser.add_argument("--video-device", type=int, default=0, help="OpenCV device index")
    args = parser.parse_args()

    app = create_app(
        target_ip=args.ip,
        target_port=args.port,
        video_device=args.video_device,
    )

    print(f"MJPEG preview → http://{args.flask_host}:{args.flask_port}/video")
    print(f"RTP target    → udp://{args.ip}:{args.port}")
    print()

    # TODO: Replace with a production WSGI server (gunicorn / waitress) for deployment.
    app.run(host=args.flask_host, port=args.flask_port, threaded=True)


if __name__ == "__main__":
    main()
