# dune detection module
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional

logger = logging.getLogger(__name__)

try:
    from MainCode.ultrasonic import get_distance
except Exception:
    def get_distance() -> float:  # ignore
        raise ImportError("ultrasonic.get_distance not available")

try:
    from MainCode.ir_sensor import is_obstacle
except Exception:
    def is_obstacle() -> bool:  # ignore
        raise ImportError("ir_sensor.is_obstacle not available")

try:
    from MainCode.camera_module import capture_frame
except Exception:
    def capture_frame() -> Any:  # ignore
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
    # return camera confidence 0..1
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

    # brightness heuristic
    if arr.size == 0:
        return 0.0

    # convert to grayscale proxy
    if arr.ndim == 3:
        gray = arr.mean(axis=2)
    else:
        gray = arr

    mean = float(gray.mean())
    # map mean (0..255) to 0..1
    confidence = mean / 255.0

    # texture proxy
    std = float(gray.std())
    texture_factor = 1.0 - _clamp01(std / 64.0)

    return _clamp01(0.6 * confidence + 0.4 * texture_factor)


def detect_dune(distance_threshold: float = 1.0, use_camera: bool = True) -> DetectionResult:
        # combine sensors into a detection result
    distance = float("inf")
    obstacle = False
    frame = None
    camera_conf = 0.0

    # read ultrasonic
    try:
        distance = float(get_distance())
    except Exception as exc:  # pragma: no cover - hardware dependent
        logger.debug("ultrasonic read failed: %s", exc)

    # read ir
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

    # combine heuristics
    conf = camera_conf * 0.7
    if distance < distance_threshold:
        conf += 0.25
    if obstacle:
        conf += 0.15

    conf = _clamp01(conf)

    return DetectionResult(distance=distance, obstacle=obstacle, dune_confidence=conf, frame=frame)
