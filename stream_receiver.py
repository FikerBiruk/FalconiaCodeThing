#!/usr/bin/env python3
"""GStreamer receiver – display the Falconia H.264/RTP stream on a client.

This script runs on the **receiving** machine (laptop, desktop, etc.) and
opens a video window showing the live feed from the rover.

Usage
-----
    # Listen on the default port
    python stream_receiver.py

    # Custom port
    python stream_receiver.py --port 5000

Pipeline
--------
    udpsrc port=5000
    ! application/x-rtp,encoding-name=H264,payload=96
    ! rtpjitterbuffer                     ← smooths network jitter
    ! rtph264depay                        ← extracts H.264 from RTP
    ! avdec_h264                          ← decodes H.264 to raw video
    ! videoconvert                        ← colourspace for the video sink
    ! autovideosink                       ← platform-native display window

Requires
--------
    GStreamer 1.x with gst-plugins-good, gst-plugins-bad, gst-libav.
"""
from __future__ import annotations

import argparse
import logging
import shutil
import subprocess
import signal
import sys

logger = logging.getLogger("falconia.receiver")


def build_receiver_command(port: int = 5000, latency: int = 200) -> list[str]:
    """Build the ``gst-launch-1.0`` command list for receiving a stream.

    Parameters
    ----------
    port : int
        UDP port to listen on.
    latency : int
        Jitter-buffer latency in milliseconds.
    """
    return [
        "gst-launch-1.0", "-e",
        # udpsrc: listen for incoming RTP packets
        "udpsrc", f"port={port}",
        "!",
        # Caps filter: tell GStreamer what kind of data to expect
        f"application/x-rtp,encoding-name=H264,payload=96",
        "!",
        # rtpjitterbuffer: re-order and buffer packets to handle network jitter
        "rtpjitterbuffer", f"latency={latency}",
        "!",
        # rtph264depay: extract the H.264 NALUs from RTP packets
        "rtph264depay",
        "!",
        # avdec_h264: decode H.264 → raw video frames
        "avdec_h264",
        "!",
        # videoconvert: convert pixel formats for the display sink
        "videoconvert",
        "!",
        # autovideosink: display in a native window (X11, Wayland, DirectShow…)
        "autovideosink", "sync=false",
    ]


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Receive and display an H.264/RTP stream from the Falconia rover."
    )
    parser.add_argument("--port", type=int, default=5000, help="UDP port to listen on")
    parser.add_argument("--latency", type=int, default=200, help="Jitter-buffer latency (ms)")
    args = parser.parse_args()

    # Pre-flight check
    if not shutil.which("gst-launch-1.0"):
        logger.error("gst-launch-1.0 not found. Install GStreamer first.")
        sys.exit(1)

    cmd = build_receiver_command(port=args.port, latency=args.latency)
    logger.info("Starting receiver on UDP port %d …", args.port)
    logger.debug("Command: %s", " ".join(cmd))

    try:
        proc = subprocess.Popen(cmd)
        proc.wait()
    except KeyboardInterrupt:
        logger.info("Interrupted – shutting down receiver.")
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)
    except Exception as exc:
        logger.error("Receiver failed: %s", exc)
        sys.exit(1)

    sys.exit(proc.returncode or 0)


if __name__ == "__main__":
    main()
