# camera helper
from __future__ import annotations

import logging
from typing import Any, Optional

logger = logging.getLogger(__name__)

_use_picam = False
_picam_instance: Optional["Picamera2"] = None

try:
	from picamera2 import Picamera2
	_use_picam = True
except Exception:
	_use_picam = False


def _init_picam() -> None:
	global _picam_instance
	if _picam_instance is None:
		_picam_instance = Picamera2()
		try:
			_picam_instance.start()
		except Exception as exc:
			logger.debug("failed to start Picamera2: %s", exc)


def capture_frame() -> Any:
	"""Capture and return a single frame (numpy array-like).

	Uses Picamera2 if available, otherwise falls back to OpenCV VideoCapture.
	"""
	try:
		import numpy as np  # ensure numpy
	except Exception:
		np = None

	if _use_picam:
		try:
			_init_picam()
			frame = _picam_instance.capture_array()
			return frame
		except Exception as exc:
			logger.debug("Picamera2 capture failed: %s", exc)

	# fallback to opencv
	try:
		import cv2
	except Exception as exc:  # pragma: no cover - runtime dependent
		raise RuntimeError("No camera backend available: install picamera2 or opencv-python") from exc

	cap = cv2.VideoCapture(0)
	if not cap.isOpened():
		cap.release()
		raise RuntimeError("Unable to open default camera (index 0)")

	ret, frame = cap.read()
	cap.release()
	if not ret:
		raise RuntimeError("Failed to read frame from camera")

	return frame


def _preview_loop() -> None:
	try:
		import cv2
	except Exception:
		raise RuntimeError("OpenCV is required for preview. Install opencv-python.")

	if _use_picam:
		_init_picam()

	win = "camera_preview"
	cv2.namedWindow(win, cv2.WINDOW_NORMAL)
	while True:
		try:
			frame = capture_frame()
		except Exception as exc:
			logger.error("capture_frame error: %s", exc)
			break

		if frame is None:
			logger.error("No frame returned from capture_frame()")
			break

			# try convert frame
		try:
			cv2.imshow(win, frame)
		except Exception:
			import numpy as np
			cv2.imshow(win, np.asarray(frame))

		key = cv2.waitKey(1) & 0xFF
			if key == ord('q') or key == 27:  # q or esc
			break

	cv2.destroyAllWindows()


if __name__ == "__main__":
	logging.basicConfig(level=logging.INFO)
	_preview_loop()
