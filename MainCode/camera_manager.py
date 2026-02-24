#!/usr/bin/env python3
"""CameraManager – unified GStreamer video streaming for the Falconia rover.

Automatically detects whether a Pi Camera (libcamera) or USB webcam is
available.  Builds the appropriate GStreamer/libcamera-vid pipeline and
streams H.264 over RTP/UDP to a configurable target.

Usage
-----
    from MainCode.camera_manager import CameraManager

    cam = CameraManager(target_ip="192.168.1.100", target_port=5000)
    cam.start_stream()
    # ...
    cam.stop_stream()

Requires
--------
* GStreamer 1.x with gst-plugins-good (``rtph264pay``, ``udpsink``).
* For Pi Camera: ``rpicam-vid`` CLI (ships with Raspberry Pi OS Bookworm+).
  Falls back to ``libcamera-vid`` on older installations.
* For USB camera: GStreamer ``v4l2src`` + ``x264enc`` **or** ``v4l2h264enc``.
"""
from __future__ import annotations

import enum
import logging
import os
import shutil
import subprocess
import signal
import threading
import time
from pathlib import Path
from typing import Optional

logger = logging.getLogger("falconia.camera_manager")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30
DEFAULT_BITRATE = 500          # kbit/s for x264enc
DEFAULT_PORT = 5000
DEFAULT_HOST = "127.0.0.1"

# How long to wait (seconds) before retrying after a pipeline crash.
RECONNECT_DELAY = 3.0
MAX_RECONNECT_ATTEMPTS = 5


# ---------------------------------------------------------------------------
# Camera type enumeration
# ---------------------------------------------------------------------------

class CameraType(enum.Enum):
    """Detected camera backend."""
    PI_CAMERA = "pi_camera"
    USB_CAMERA = "usb_camera"
    NONE = "none"


# ---------------------------------------------------------------------------
# Detection helpers
# ---------------------------------------------------------------------------

def _find_video_devices() -> list[str]:
    """Return a sorted list of ``/dev/videoN`` device paths."""
    dev = Path("/dev")
    return sorted(str(p) for p in dev.glob("video*") if p.is_char_device())


def _get_rpicam_command() -> Optional[str]:
    """Return the Pi Camera CLI name available on this system.

    Prefers ``rpicam-vid`` (Bookworm+); falls back to ``libcamera-vid``.
    Returns *None* if neither is found.
    """
    for cmd in ("rpicam-vid", "libcamera-vid"):
        if shutil.which(cmd) is not None:
            return cmd
    return None


def _is_libcamera_available() -> bool:
    """Check whether ``rpicam-vid`` or ``libcamera-vid`` is installed."""
    return _get_rpicam_command() is not None


def _has_usb_camera() -> bool:
    """Return *True* if at least one V4L2 video device exists."""
    return len(_find_video_devices()) > 0


def detect_camera_type() -> CameraType:
    """Auto-detect the best available camera backend.

    Priority:
        1. Pi Camera via libcamera (preferred on Raspberry Pi).
        2. USB camera via V4L2.
        3. ``CameraType.NONE`` if nothing useful is found.
    """
    if _is_libcamera_available():
        logger.info("%s found – selecting Pi Camera backend.", _get_rpicam_command())
        return CameraType.PI_CAMERA
    if _has_usb_camera():
        devices = _find_video_devices()
        logger.info("USB camera detected (%s).", ", ".join(devices))
        return CameraType.USB_CAMERA
    logger.warning("No camera backend detected.")
    return CameraType.NONE


# ---------------------------------------------------------------------------
# Pipeline builders
# ---------------------------------------------------------------------------

def build_picamera_pipeline(
    target_ip: str,
    target_port: int,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    fps: int = DEFAULT_FPS,
) -> list[list[str]]:
    """Return a two-stage pipeline list for Pi Camera streaming.

    Stage 1 – ``rpicam-vid`` (or ``libcamera-vid``) captures H.264 from
    the Pi Camera and writes raw NAL units to *stdout*.

    Stage 2 – ``gst-launch-1.0`` reads from *stdin* (``fdsrc``), parses
    the H.264 stream, wraps it in RTP, and sends it over UDP.

    The two stages are connected via a Unix pipe (stdout → stdin).
    """
    # Resolve the correct CLI name (rpicam-vid preferred, libcamera-vid fallback)
    rpicam_cmd = _get_rpicam_command()
    if rpicam_cmd is None:
        raise RuntimeError(
            "Neither rpicam-vid nor libcamera-vid found on $PATH. "
            "Install with: sudo apt install rpicam-apps"
        )

    # Stage 1: rpicam-vid  →  stdout
    libcamera_cmd = [
        rpicam_cmd,
        "-t", "0",                        # run indefinitely
        "--width", str(width),
        "--height", str(height),
        "--framerate", str(fps),
        "--inline",                        # emit SPS/PPS with every IDR
        "-n",                              # no preview window
        "--codec", "h264",
        "--output", "-",                   # write to stdout
    ]

    # Stage 2: GStreamer reads from fd 0 (stdin)
    gst_cmd = [
        "gst-launch-1.0", "-e",
        # fdsrc: reads raw bytes from a file-descriptor (stdin here)
        "fdsrc",
        "!",
        # h264parse: re-frames / timestamps the raw H.264 NALUs
        "h264parse",
        "!",
        # rtph264pay: packetises H.264 into RTP for network transport
        "rtph264pay", "config-interval=1", "pt=96",
        "!",
        # udpsink: sends RTP packets over UDP to the target
        "udpsink", f"host={target_ip}", f"port={target_port}",
    ]

    return [libcamera_cmd, gst_cmd]


def build_usb_camera_pipeline(
    target_ip: str,
    target_port: int,
    device: str = "/dev/video0",
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    fps: int = DEFAULT_FPS,
    bitrate: int = DEFAULT_BITRATE,
) -> list[str]:
    """Return a single ``gst-launch-1.0`` command for USB camera streaming.

    Pipeline:
        v4l2src → videoconvert → x264enc (software, zero-latency) →
        rtph264pay → udpsink

    NOTE: For hardware encoding on a Pi, replace ``x264enc`` with
    ``v4l2h264enc`` if available.  This is a TODO for future optimisation.
    """
    return [
        "gst-launch-1.0", "-e",
        # v4l2src: captures frames from a V4L2 device
        "v4l2src", f"device={device}",
        "!",
        # Negotiate a specific resolution / framerate from the source
        f"video/x-raw,width={width},height={height},framerate={fps}/1",
        "!",
        # videoconvert: colourspace conversion to a format x264enc accepts
        "videoconvert",
        "!",
        # x264enc: software H.264 encoder, tuned for lowest latency
        "x264enc",
        f"bitrate={bitrate}",
        "tune=zerolatency",
        "speed-preset=superfast",
        "!",
        # rtph264pay: packetise H.264 into RTP
        "rtph264pay", "config-interval=1", "pt=96",
        "!",
        # udpsink: send RTP over UDP
        "udpsink", f"host={target_ip}", f"port={target_port}",
    ]


# ---------------------------------------------------------------------------
# CameraManager
# ---------------------------------------------------------------------------

class CameraManager:
    """High-level controller for starting / stopping a video stream.

    Parameters
    ----------
    target_ip : str
        IP address of the receiving client.
    target_port : int
        UDP port on the receiving client.
    camera_type : CameraType | None
        Force a specific backend.  *None* means auto-detect.
    device : str
        V4L2 device path when using a USB camera.
    width, height, fps, bitrate : int
        Stream parameters.
    auto_reconnect : bool
        If *True*, automatically restart the pipeline on failure.
    """

    def __init__(
        self,
        target_ip: str = DEFAULT_HOST,
        target_port: int = DEFAULT_PORT,
        camera_type: Optional[CameraType] = None,
        device: str = "/dev/video0",
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: int = DEFAULT_FPS,
        bitrate: int = DEFAULT_BITRATE,
        auto_reconnect: bool = True,
    ) -> None:
        self.target_ip = target_ip
        self.target_port = target_port
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        self.auto_reconnect = auto_reconnect

        # Detect camera if not explicitly provided.
        self.camera_type = camera_type if camera_type is not None else detect_camera_type()

        # Internal state
        self._processes: list[subprocess.Popen] = []
        self._watchdog_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._reconnect_attempts = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_streaming(self) -> bool:
        """Return *True* if the pipeline process(es) are alive."""
        return any(p.poll() is None for p in self._processes)

    def start_stream(self) -> None:
        """Build and launch the appropriate GStreamer pipeline.

        Raises
        ------
        RuntimeError
            If no camera backend is available or the pipeline fails to start.
        """
        if self.is_streaming:
            logger.warning("Stream already running – call stop_stream() first.")
            return

        if self.camera_type == CameraType.NONE:
            raise RuntimeError(
                "No camera detected. Attach a USB webcam or enable the Pi Camera."
            )

        self._stop_event.clear()
        self._reconnect_attempts = 0
        self._launch_pipeline()

        # Start a background watchdog that monitors the pipeline health.
        self._watchdog_thread = threading.Thread(
            target=self._watchdog, daemon=True, name="camera-watchdog"
        )
        self._watchdog_thread.start()

        logger.info(
            "Streaming %s → udp://%s:%d  (%dx%d @ %dfps)",
            self.camera_type.value,
            self.target_ip,
            self.target_port,
            self.width,
            self.height,
            self.fps,
        )

    def stop_stream(self) -> None:
        """Gracefully terminate all pipeline processes."""
        self._stop_event.set()
        self._terminate_processes()

        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=5)
            self._watchdog_thread = None

        logger.info("Stream stopped.")

    def restart_stream(self) -> None:
        """Stop and re-start the stream."""
        self.stop_stream()
        time.sleep(0.5)
        self.start_stream()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _launch_pipeline(self) -> None:
        """Spawn the correct subprocess(es) for the detected camera type."""
        self._terminate_processes()

        if self.camera_type == CameraType.PI_CAMERA:
            stages = build_picamera_pipeline(
                self.target_ip, self.target_port,
                self.width, self.height, self.fps,
            )
            # Pipe libcamera-vid stdout into gst-launch-1.0 stdin.
            libcamera_proc = subprocess.Popen(
                stages[0],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            gst_proc = subprocess.Popen(
                stages[1],
                stdin=libcamera_proc.stdout,
                stderr=subprocess.PIPE,
            )
            # Allow libcamera_proc to receive SIGPIPE if gst_proc exits.
            if libcamera_proc.stdout is not None:
                libcamera_proc.stdout.close()
            self._processes = [libcamera_proc, gst_proc]

        elif self.camera_type == CameraType.USB_CAMERA:
            cmd = build_usb_camera_pipeline(
                self.target_ip, self.target_port,
                self.device, self.width, self.height,
                self.fps, self.bitrate,
            )
            proc = subprocess.Popen(cmd, stderr=subprocess.PIPE)
            self._processes = [proc]

        else:
            raise RuntimeError(f"Unsupported camera type: {self.camera_type}")

    def _terminate_processes(self) -> None:
        """Send SIGINT then SIGKILL to any running pipeline processes."""
        for proc in self._processes:
            if proc.poll() is None:
                try:
                    # SIGINT lets gst-launch-1.0 perform EOS shutdown.
                    proc.send_signal(signal.SIGINT)
                    proc.wait(timeout=3)
                except (subprocess.TimeoutExpired, OSError):
                    proc.kill()
                    proc.wait(timeout=2)
        self._processes.clear()

    def _watchdog(self) -> None:
        """Background thread that monitors pipeline health and reconnects."""
        while not self._stop_event.is_set():
            time.sleep(1.0)

            if self._stop_event.is_set():
                break

            if not self.is_streaming:
                # Collect stderr from crashed processes for diagnostics.
                for proc in self._processes:
                    stderr_data = b""
                    if proc.stderr is not None:
                        try:
                            stderr_data = proc.stderr.read() or b""
                        except Exception:
                            pass
                    rc = proc.returncode
                    logger.error(
                        "Pipeline process (PID %d) exited with code %s. stderr: %s",
                        proc.pid, rc, stderr_data.decode(errors="replace")[:500],
                    )

                if not self.auto_reconnect:
                    logger.info("Auto-reconnect disabled – giving up.")
                    break

                self._reconnect_attempts += 1
                if self._reconnect_attempts > MAX_RECONNECT_ATTEMPTS:
                    logger.error(
                        "Exceeded %d reconnect attempts – giving up.",
                        MAX_RECONNECT_ATTEMPTS,
                    )
                    break

                logger.info(
                    "Reconnect attempt %d/%d in %.1fs …",
                    self._reconnect_attempts, MAX_RECONNECT_ATTEMPTS,
                    RECONNECT_DELAY,
                )
                time.sleep(RECONNECT_DELAY)
                try:
                    self._launch_pipeline()
                except Exception as exc:
                    logger.error("Reconnect failed: %s", exc)

    # ------------------------------------------------------------------
    # Context-manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "CameraManager":
        self.start_stream()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.stop_stream()

    # ------------------------------------------------------------------
    # Diagnostics (class-level helpers)
    # ------------------------------------------------------------------

    @staticmethod
    def list_video_devices() -> list[str]:
        """Return available ``/dev/videoN`` paths."""
        return _find_video_devices()

    @staticmethod
    def check_gstreamer() -> bool:
        """Return *True* if ``gst-launch-1.0`` is on ``$PATH``."""
        found = shutil.which("gst-launch-1.0") is not None
        if not found:
            logger.warning("gst-launch-1.0 not found – install gstreamer1.0-tools.")
        return found

    @staticmethod
    def check_libcamera() -> bool:
        """Return *True* if ``rpicam-vid`` or ``libcamera-vid`` is on ``$PATH``."""
        return _is_libcamera_available()


# ---------------------------------------------------------------------------
# CLI quick-start
# ---------------------------------------------------------------------------

def _cli() -> None:
    """Minimal CLI for quick testing: ``python -m MainCode.camera_manager``."""
    import argparse

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="Falconia camera streamer")
    parser.add_argument("--ip", default=DEFAULT_HOST, help="Target IP address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Target UDP port")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument(
        "--camera", choices=["pi", "usb", "auto"], default="auto",
        help="Force camera backend",
    )
    args = parser.parse_args()

    cam_type: Optional[CameraType] = None
    if args.camera == "pi":
        cam_type = CameraType.PI_CAMERA
    elif args.camera == "usb":
        cam_type = CameraType.USB_CAMERA

    cam = CameraManager(
        target_ip=args.ip,
        target_port=args.port,
        camera_type=cam_type,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )

    print("Detected devices:", cam.list_video_devices())
    print("GStreamer OK:    ", cam.check_gstreamer())
    print("libcamera OK:    ", cam.check_libcamera())
    print()

    try:
        cam.start_stream()
        print("Streaming… press Ctrl+C to stop.\n")
        while cam.is_streaming:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop_stream()


if __name__ == "__main__":
    _cli()
