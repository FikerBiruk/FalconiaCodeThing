"""dune_detection

High-level detection logic that combines ultrasonic, IR, and camera modules
into a single `detect_dune()` function. The implementation below uses simple
heuristics and is intentionally modular so you can replace `analyze_frame`
with a proper ML model or more advanced image processing later.

Expectations for sensor modules (must be provided elsewhere in the project):
- `ultrasonic.get_distance() -> float` (meters)
- `ir_sensor.is_obstacle() -> bool`
- `camera_module.capture_frame() -> Any` (e.g. numpy array / PIL Image)

"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional

logger = logging.getLogger(__name__)

try:
    from ultrasonic import get_distance
except Exception:  # pragma: no cover - sensors may be absent on dev machine
    def get_distance() -> float:  # type: ignore
        raise ImportError("ultrasonic.get_distance not available")

try:
    from ir_sensor import is_obstacle
except Exception:  # pragma: no cover
    def is_obstacle() -> bool:  # type: ignore
        raise ImportError("ir_sensor.is_obstacle not available")

try:
    from camera_module import capture_frame
except Exception:  # pragma: no cover
    def capture_frame() -> Any:  # type: ignore
        raise ImportError("camera_module.capture_frame not available")


@dataclass
class DetectionResult:
    distance: float
    obstacle: bool
    dune_confidence: float  # 0.0 - 1.0
    frame: Optional[Any] = None


def _clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))


def analyze_frame(frame: Any) -> float:
    """Return a heuristic confidence [0,1] that the frame shows a dune.

    This function uses a very small, dependency-light heuristic: if numpy
    and cv2 are available we compute a brightness / texture proxy. Replace
    this with ML or domain-specific image processing as needed.
    """
    try:
        import numpy as np
    except Exception:
        logger.debug("numpy not available; falling back to neutral camera confidence")
        return 0.4

    try:
        arr = np.asarray(frame)
    except Exception:
        logger.debug("could not convert frame to array")
        return 0.4

    # Brightness heuristic: dunes often have high-average intensity in visible
    # spectrum frames (very approximate). Compute mean brightness and normalize.
    if arr.size == 0:
        return 0.0

    # If it's color, convert to grayscale proxy
    if arr.ndim == 3:
        gray = arr.mean(axis=2)
    else:
        gray = arr

    mean = float(gray.mean())
    # map mean (0..255) to 0..1
    confidence = mean / 255.0

    # Texture proxy: dunes may be relatively smooth vs vegetation; use std dev
    std = float(gray.std())
    texture_factor = 1.0 - _clamp01(std / 64.0)

    return _clamp01(0.6 * confidence + 0.4 * texture_factor)


def detect_dune(distance_threshold: float = 1.0, use_camera: bool = True) -> DetectionResult:
    """Combine sensor readings and return a `DetectionResult`.

    - `distance_threshold` (meters): if ultrasonic distance is less than this,
      the environment could indicate a dune or nearby obstacle.
    - `use_camera`: attempt to call `capture_frame()` when True; safe to
      disable for faster runs or when camera hardware is absent.
    """
    distance = float("inf")
    obstacle = False
    frame = None
    camera_conf = 0.0

    # read ultrasonic
    try:
        distance = float(get_distance())
    except Exception as exc:  # pragma: no cover - hardware dependent
        logger.debug("ultrasonic read failed: %s", exc)

    # read IR
    try:
        obstacle = bool(is_obstacle())
    except Exception as exc:  # pragma: no cover
        logger.debug("ir sensor read failed: %s", exc)

    # read camera
    if use_camera:
        try:
            frame = capture_frame()
            camera_conf = analyze_frame(frame)
        except Exception as exc:  # pragma: no cover
            logger.debug("camera capture/analyze failed: %s", exc)

    # Combine heuristics: camera has primary vote, distance and IR adjust it.
    conf = camera_conf * 0.7
    if distance < distance_threshold:
        conf += 0.25
    if obstacle:
        conf += 0.15

    conf = _clamp01(conf)

    return DetectionResult(distance=distance, obstacle=obstacle, dune_confidence=conf, frame=frame)
