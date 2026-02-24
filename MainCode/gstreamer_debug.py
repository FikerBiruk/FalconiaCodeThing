#!/usr/bin/env python3
"""Debugging helpers for the Falconia GStreamer video pipeline.

Run standalone to print a full diagnostic report::

    python -m MainCode.gstreamer_debug

Or import individual checks in other modules::

    from MainCode.gstreamer_debug import full_diagnostic
    full_diagnostic()
"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
from pathlib import Path
from typing import Optional

logger = logging.getLogger("falconia.gstreamer_debug")

# ---------------------------------------------------------------------------
# Device inspection
# ---------------------------------------------------------------------------

def list_video_devices() -> list[dict[str, str]]:
    """Return metadata for each ``/dev/videoN`` device.

    Each dict contains:
        - ``path``:  e.g. ``/dev/video0``
        - ``name``:  human-readable device name (from ``v4l2-ctl``) or ``"unknown"``
        - ``driver``: kernel driver name or ``"unknown"``
    """
    dev = Path("/dev")
    devices: list[dict[str, str]] = []

    for devpath in sorted(dev.glob("video*")):
        if not devpath.is_char_device():
            continue
        info: dict[str, str] = {"path": str(devpath), "name": "unknown", "driver": "unknown"}

        # Try v4l2-ctl for richer metadata
        if shutil.which("v4l2-ctl"):
            try:
                out = subprocess.check_output(
                    ["v4l2-ctl", "-d", str(devpath), "--info"],
                    stderr=subprocess.DEVNULL,
                    timeout=5,
                ).decode(errors="replace")
                for line in out.splitlines():
                    if "Card type" in line:
                        info["name"] = line.split(":", 1)[1].strip()
                    if "Driver name" in line:
                        info["driver"] = line.split(":", 1)[1].strip()
            except Exception:
                pass

        devices.append(info)

    return devices


# ---------------------------------------------------------------------------
# GStreamer availability
# ---------------------------------------------------------------------------

def check_gstreamer_install() -> dict[str, object]:
    """Verify that GStreamer is installed and report its version.

    Returns a dict with:
        - ``installed``: bool
        - ``version``: str | None
        - ``path``: str | None   (path to gst-launch-1.0)
        - ``plugins``: list[str] (inspected plugin names, best-effort)
    """
    result: dict[str, object] = {
        "installed": False,
        "version": None,
        "path": None,
        "plugins": [],
    }

    gst_path = shutil.which("gst-launch-1.0")
    if gst_path is None:
        return result

    result["installed"] = True
    result["path"] = gst_path

    # Version
    try:
        out = subprocess.check_output(
            ["gst-launch-1.0", "--version"],
            stderr=subprocess.DEVNULL,
            timeout=5,
        ).decode(errors="replace")
        for line in out.splitlines():
            if "GStreamer" in line:
                result["version"] = line.strip()
                break
    except Exception:
        pass

    # Key plugins we rely on
    required_plugins = [
        "v4l2src", "videoconvert", "x264enc", "rtph264pay",
        "udpsink", "udpsrc", "h264parse", "fdsrc",
        "rtph264depay", "avdec_h264", "autovideosink",
        "rtpjitterbuffer",
    ]
    found: list[str] = []
    gst_inspect = shutil.which("gst-inspect-1.0")
    if gst_inspect:
        for plugin in required_plugins:
            try:
                subprocess.check_call(
                    [gst_inspect, plugin],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5,
                )
                found.append(plugin)
            except Exception:
                pass
    result["plugins"] = found
    return result


# ---------------------------------------------------------------------------
# rpicam / libcamera availability
# ---------------------------------------------------------------------------

def check_libcamera() -> dict[str, object]:
    """Check whether ``rpicam-vid`` or ``libcamera-vid`` is available.

    Prefers **rpicam-vid** (Raspberry Pi OS Bookworm+), falls back to the
    older ``libcamera-vid`` name.

    Returns a dict with:
        - ``available``: bool
        - ``command``: str | None   (the resolved CLI name)
        - ``path``: str | None
        - ``cameras``: list[str]  (output of ``--list-cameras``, best-effort)
    """
    result: dict[str, object] = {"available": False, "command": None, "path": None, "cameras": []}

    # Try the newer name first, then the legacy name.
    cmd_name: str | None = None
    for candidate in ("rpicam-vid", "libcamera-vid"):
        path = shutil.which(candidate)
        if path is not None:
            cmd_name = candidate
            break

    if cmd_name is None or path is None:
        return result

    result["available"] = True
    result["command"] = cmd_name
    result["path"] = path

    try:
        out = subprocess.check_output(
            [cmd_name, "--list-cameras"],
            stderr=subprocess.STDOUT,
            timeout=10,
        ).decode(errors="replace")
        result["cameras"] = [l.strip() for l in out.splitlines() if l.strip()]
    except Exception:
        pass

    return result


# ---------------------------------------------------------------------------
# Pipeline quick-tests
# ---------------------------------------------------------------------------

def test_pipeline(pipeline_str: str, timeout: float = 5.0) -> dict[str, object]:
    """Launch a GStreamer pipeline string and let it run for *timeout* seconds.

    Returns:
        - ``success``: bool
        - ``returncode``: int | None
        - ``stderr``: str   (first 1 000 chars)
    """
    cmd = ["gst-launch-1.0"] + pipeline_str.split()
    result: dict[str, object] = {"success": False, "returncode": None, "stderr": ""}
    try:
        proc = subprocess.Popen(cmd, stderr=subprocess.PIPE)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Pipeline kept running – that's considered a success.
            proc.terminate()
            proc.wait(timeout=3)
            result["success"] = True
        result["returncode"] = proc.returncode
        stderr_data = proc.stderr.read() if proc.stderr else b""
        result["stderr"] = stderr_data.decode(errors="replace")[:1000]
        if proc.returncode == 0:
            result["success"] = True
    except FileNotFoundError:
        result["stderr"] = "gst-launch-1.0 not found"
    except Exception as exc:
        result["stderr"] = str(exc)
    return result


def test_usb_camera_pipeline(device: str = "/dev/video0") -> dict[str, object]:
    """Quick 5-second test of the USB camera capture pipeline (no network)."""
    pipeline = (
        f"v4l2src device={device} num-buffers=150 "
        "! videoconvert ! autovideosink"
    )
    return test_pipeline(pipeline, timeout=6.0)


def test_videotestsrc_pipeline() -> dict[str, object]:
    """Validate basic GStreamer operation with a synthetic test source."""
    pipeline = "videotestsrc num-buffers=90 ! autovideosink"
    return test_pipeline(pipeline, timeout=5.0)


# ---------------------------------------------------------------------------
# Full diagnostic
# ---------------------------------------------------------------------------

def full_diagnostic() -> dict[str, object]:
    """Run all checks and return a combined report dict."""
    report: dict[str, object] = {}
    report["video_devices"] = list_video_devices()
    report["gstreamer"] = check_gstreamer_install()
    report["libcamera"] = check_libcamera()
    report["test_videotestsrc"] = test_videotestsrc_pipeline()
    return report


def print_diagnostic() -> None:
    """Pretty-print the full diagnostic to the console."""
    import json
    report = full_diagnostic()
    print("=" * 60)
    print("  Falconia GStreamer Diagnostic Report")
    print("=" * 60)
    print(json.dumps(report, indent=2, default=str))
    print("=" * 60)

    # Actionable hints
    gst = report.get("gstreamer", {})
    if not gst.get("installed"):
        print("\n[!] GStreamer is NOT installed.")
        print("    Install with:  sudo apt install gstreamer1.0-tools "
              "gstreamer1.0-plugins-good gstreamer1.0-plugins-bad "
              "gstreamer1.0-plugins-ugly")

    required = {
        "v4l2src", "videoconvert", "x264enc", "rtph264pay",
        "udpsink", "h264parse", "fdsrc", "rtph264depay",
        "avdec_h264", "autovideosink", "rtpjitterbuffer",
    }
    found = set(gst.get("plugins", []))
    missing = required - found
    if missing:
        print(f"\n[!] Missing GStreamer elements: {', '.join(sorted(missing))}")
        print("    You may need gst-plugins-good, gst-plugins-bad, or gst-plugins-ugly.")

    libcam = report.get("libcamera", {})
    if not libcam.get("available"):
        print("\n[i] libcamera-vid not found – Pi Camera streaming will be unavailable.")

    devs = report.get("video_devices", [])
    if not devs:
        print("\n[!] No /dev/video* devices found.")
    else:
        print(f"\n[+] Found {len(devs)} video device(s):")
        for d in devs:
            print(f"    {d['path']}  ({d['name']}, driver={d['driver']})")


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    print_diagnostic()
