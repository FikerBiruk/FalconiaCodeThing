"""Main runner for the rover's dune detection loop.

This script polls `detect_dune()` and logs the result. It intentionally
keeps motor/action integration as placeholders so you can wire your
`motor_controls` implementation safely.
"""
from __future__ import annotations

import logging
import time
from typing import Optional

from dune_detection import detect_dune, DetectionResult

logger = logging.getLogger("rover")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")


def take_action(result: DetectionResult) -> None:
    """Decide motor actions based on detection result.

    Replace the placeholder comments with calls to your `motor_controls`
    functions (e.g., `motor_controls.drive_forward()`, `stop()`, etc.).
    """
    if result.dune_confidence >= 0.6:
        logger.info("Dune likely (%.2f). Stopping / avoiding.", result.dune_confidence)
        # TODO: integrate motor_controls to stop or navigate around dune
        # import motor_controls
        # motor_controls.stop()
    else:
        logger.info("No dune (%.2f). Continue exploring.", result.dune_confidence)
        # TODO: motor_controls.drive_forward()


def main(loop_delay: float = 1.0, use_camera: bool = True, distance_threshold: float = 1.0) -> None:
    logger.info("Starting rover loop (camera=%s, distance_threshold=%.2fm)", use_camera, distance_threshold)
    try:
        while True:
            result = detect_dune(distance_threshold=distance_threshold, use_camera=use_camera)
            logger.info("distance=%.2fm obstacle=%s confidence=%.2f", result.distance, result.obstacle, result.dune_confidence)
            take_action(result)
            time.sleep(loop_delay)
    except KeyboardInterrupt:
        logger.info("Shutting down rover loop")


if __name__ == "__main__":
    main()
