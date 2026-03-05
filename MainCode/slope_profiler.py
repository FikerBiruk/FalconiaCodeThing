#!/usr/bin/env python3
"""
Ultrasonic Slope Profiler for Raspberry Pi
==========================================

Wiring (HC-SR04):
  - VCC  → 5V power (Pin 2 or 4)
  - GND  → Ground (Pin 6, 9, 14, 20, 25, 30, 34, or 39)
  - TRIG → GPIO 23 (Pin 16) — 3.3V-safe digital output from the Pi
  - ECHO → GPIO 24 (Pin 18) — *** USE A VOLTAGE DIVIDER! ***
      The ECHO pin outputs 5V but the Pi GPIO is 3.3V-max.
      Use a 1kΩ + 2kΩ resistive divider:
         ECHO pin → 1kΩ resistor → junction → 2kΩ resistor → GND
         Connect junction to GPIO 24.
      This drops 5V to ~3.3V safely.

Safety Notes:
  - Never connect ECHO directly to a GPIO pin without a voltage divider.
  - HC-SR04 operating range: 2 cm – 400 cm; readings outside this are unreliable.
  - Add a 10 µF capacitor across VCC-GND near the sensor to smooth power spikes.
  - Maximum recommended sampling rate: ~40 Hz (each ping cycle ≈ 25 ms).
  - Keep the sensor away from motors/servos to reduce ultrasonic interference.

Usage:
  CLI mode (live CSV output):
    python slope_profiler.py --cli --rate 10 --window 20

  Calibrate at a known 30 cm surface:
    python slope_profiler.py --calibrate --cal-distance 30 --cal-samples 50

  Run simulation / unit tests:
    python slope_profiler.py --test

  Desktop demo (no Pi hardware):
    python slope_profiler.py --cli
    (automatically uses a mock reader when RPi.GPIO is unavailable)
"""

import time
import math
import sys
import argparse
import collections
from typing import Callable, List, Optional, Tuple, NamedTuple


# ---------------------------------------------------------------------------
# GPIO abstraction with mock fallback for desktop testing
# ---------------------------------------------------------------------------

try:
    import RPi.GPIO as GPIO
    _ON_PI = True
except ImportError:
    _ON_PI = False

    class _MockGPIO:
        """Fallback mock GPIO class for desktop development and testing.

        Simulates BCM pin numbering, pin setup, and basic trigger/echo
        timing so that the code runs without real hardware.
        """
        BCM = 11
        OUT = 0
        IN = 1
        HIGH = 1
        LOW = 0

        @staticmethod
        def setmode(mode):
            pass

        @staticmethod
        def setup(pin, direction):
            pass

        @staticmethod
        def output(pin, value):
            pass

        @staticmethod
        def input(pin):
            return _MockGPIO.LOW

        @staticmethod
        def cleanup():
            pass

        @staticmethod
        def setwarnings(flag):
            pass

    GPIO = _MockGPIO  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class SlopeReading(NamedTuple):
    """A single slope-profile measurement."""
    timestamp: float          # epoch seconds
    distance_cm: float        # bias-corrected distance in cm
    slope_deg: float          # instantaneous slope angle in degrees
    slope_rise_run: float     # rise/run = Δdistance / Δtime (cm/s)
    smoothed_slope_deg: float # EMA-smoothed slope in degrees
    confidence: float         # 0.0–1.0 quality metric
    condition: str            # flat | gentle_slope | steep_slope | vertical


class CalibrationResult(NamedTuple):
    """Output of the calibration routine."""
    mean_cm: float            # mean measured distance
    stddev_cm: float          # standard deviation of samples
    bias_cm: float            # mean − expected (systematic error)
    samples: int              # number of valid samples collected


# ---------------------------------------------------------------------------
# Pure-Python linear regression (no numpy dependency)
# ---------------------------------------------------------------------------

def linear_regression(
    xs: List[float], ys: List[float]
) -> Tuple[float, float, float]:
    """Ordinary Least Squares (OLS) linear regression: y = slope·x + intercept.

    Math (closed-form solution):
        Given n data points (xᵢ, yᵢ):

        slope     = (n·Σxᵢyᵢ − Σxᵢ·Σyᵢ) / (n·Σxᵢ² − (Σxᵢ)²)
        intercept = (Σyᵢ − slope·Σxᵢ) / n

        Residual RMS (root-mean-square of errors):
        residual  = √( (1/n) · Σ(yᵢ − (slope·xᵢ + intercept))² )

    Args:
        xs: Independent variable values (e.g. timestamps).
        ys: Dependent variable values (e.g. distances).

    Returns:
        (slope, intercept, residual_rms)
    """
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0), 0.0

    sum_x = sum(xs)
    sum_y = sum(ys)
    sum_xy = sum(x * y for x, y in zip(xs, ys))
    sum_x2 = sum(x * x for x in xs)

    denom = n * sum_x2 - sum_x * sum_x
    if abs(denom) < 1e-12:
        # All x values identical → undefined slope; return flat
        return 0.0, sum_y / n, 0.0

    slope = (n * sum_xy - sum_x * sum_y) / denom
    intercept = (sum_y - slope * sum_x) / n

    # Root-mean-square of residuals (measures goodness of fit)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    residual_rms = math.sqrt(ss_res / n)

    return slope, intercept, residual_rms


def sample_variance(values: List[float]) -> float:
    """Compute sample variance using Bessel's correction.

    Var = Σ(xᵢ − x̄)² / (n − 1)

    Returns 0.0 for fewer than 2 values.
    """
    n = len(values)
    if n < 2:
        return 0.0
    mean = sum(values) / n
    return sum((v - mean) ** 2 for v in values) / (n - 1)


# ---------------------------------------------------------------------------
# UltrasonicReader — hardware abstraction for HC-SR04
# ---------------------------------------------------------------------------

class UltrasonicReader:
    """Reads distance from an HC-SR04 ultrasonic sensor via GPIO.

    Pulse-echo timing method:
      1. Send a 10 µs HIGH pulse on the TRIG pin.
      2. The sensor emits an 8-cycle 40 kHz ultrasonic burst.
      3. Measure how long the ECHO pin stays HIGH (round-trip time).
      4. distance_cm = (echo_duration_s × speed_of_sound_cm_s) / 2
         Speed of sound ≈ 34300 cm/s at 20 °C.
         Divide by 2 because the pulse travels to the object AND back.
    """

    SPEED_OF_SOUND_CM_S = 34300.0   # speed of sound at ~20 °C
    MIN_RANGE_CM = 2.0              # sensor minimum reliable range
    MAX_RANGE_CM = 400.0            # sensor maximum reliable range
    TIMEOUT_S = 0.04                # 40 ms ≈ max round-trip for 400 cm

    def __init__(
        self,
        trig_pin: int = 23,
        echo_pin: int = 24,
        bias_cm: float = 0.0,
    ):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.bias_cm = bias_cm  # subtracted from every raw reading

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        GPIO.output(self.trig_pin, GPIO.LOW)
        time.sleep(0.1)  # let sensor settle after power-on

    def read_distance_cm(self) -> Optional[float]:
        """Take a single distance measurement.

        Returns:
            Distance in cm (bias-corrected), or None on timeout/out-of-range.
        """
        # --- Step 1: Send 10 µs trigger pulse ---
        GPIO.output(self.trig_pin, GPIO.HIGH)
        time.sleep(0.00001)  # 10 µs
        GPIO.output(self.trig_pin, GPIO.LOW)

        # --- Step 2: Wait for echo to go HIGH (start of return pulse) ---
        start = time.monotonic()
        deadline = start + self.TIMEOUT_S

        while GPIO.input(self.echo_pin) == GPIO.LOW:
            start = time.monotonic()
            if start > deadline:
                return None  # no echo received

        # --- Step 3: Wait for echo to go LOW (end of return pulse) ---
        end = start
        while GPIO.input(self.echo_pin) == GPIO.HIGH:
            end = time.monotonic()
            if end > deadline:
                return None  # echo stuck high

        # --- Step 4: Compute distance from echo duration ---
        elapsed_s = end - start
        distance = (elapsed_s * self.SPEED_OF_SOUND_CM_S) / 2.0

        # Apply calibration bias correction
        distance -= self.bias_cm

        # Reject readings outside the sensor's reliable range
        if distance < self.MIN_RANGE_CM or distance > self.MAX_RANGE_CM:
            return None

        return round(distance, 2)

    def cleanup(self):
        """Release GPIO resources."""
        GPIO.cleanup()


class MockUltrasonicReader:
    """Mock reader for simulation and testing.

    Yields distances from a pre-loaded sequence; holds the last value
    once the sequence is exhausted.
    """

    def __init__(self, distances: Optional[List[float]] = None):
        self._distances = distances or [100.0]
        self._index = 0
        self.bias_cm = 0.0

    def read_distance_cm(self) -> Optional[float]:
        if self._index >= len(self._distances):
            return self._distances[-1]  # hold last value
        val = self._distances[self._index]
        self._index += 1
        return val

    def set_distances(self, distances: List[float]):
        """Replace the distance sequence and reset the index."""
        self._distances = distances
        self._index = 0

    def cleanup(self):
        pass


# ---------------------------------------------------------------------------
# SlopeEstimator — sliding window regression + condition detection
# ---------------------------------------------------------------------------

class SlopeEstimator:
    """Real-time slope estimator using a sliding window of distance samples.

    Algorithm:
      1. Collect (timestamp, distance_cm) pairs in a deque of size N.
      2. Run OLS linear regression:  distance = slope × time + intercept
         → slope has units cm/s (rate of distance change over time).
      3. Convert to angle:  θ = arctan(slope_cm_s) × (180 / π)
         This treats the time axis as "run" and distance-change as "rise".
      4. Apply exponential moving average (EMA) for smoothing:
         EMA_new = α × slope_deg + (1 − α) × EMA_old
         Higher α → faster response, less smoothing.
      5. Compute a confidence metric in [0, 1]:
         confidence = fill_fraction × (1 − clamp(residual / max_residual))
         where fill_fraction = current_sample_count / window_size.
      6. Classify the surface condition by slope magnitude and variance.

    Condition thresholds:
      flat          — |slope| <  5° AND variance < threshold
      gentle_slope  — 5° ≤ |slope| < 15°
      steep_slope   — 15° ≤ |slope| < 60°
      vertical      — |slope| ≥ 60° OR residual error very high
    """

    # Condition thresholds (degrees)
    FLAT_MAX_DEG = 5.0
    GENTLE_MAX_DEG = 15.0
    STEEP_MAX_DEG = 60.0
    # Maximum distance variance (cm²) to still classify as "flat"
    FLAT_MAX_VARIANCE = 4.0
    # Residual ceiling for confidence calculation (cm)
    MAX_RESIDUAL_CM = 20.0

    def __init__(self, window_size: int = 20, ema_alpha: float = 0.3, mounting_angle_deg: float = 0.0):
        """
        Args:
            window_size: Number of most-recent samples kept for regression.
            ema_alpha:   EMA smoothing factor in (0, 1].
                         0.1 = heavy smoothing, 0.9 = almost no smoothing.
            mounting_angle_deg: Sensor mounting angle in degrees (e.g. 45 for downward tilt).
                         The measured slope is corrected by subtracting this offset to get true ground slope.
        """
        self.window_size = window_size
        self.ema_alpha = ema_alpha
        self.mounting_angle_deg = mounting_angle_deg

        # Sliding window buffers (fixed max length)
        self._timestamps: collections.deque = collections.deque(maxlen=window_size)
        self._distances: collections.deque = collections.deque(maxlen=window_size)

        # EMA state
        self._smoothed_slope_deg = 0.0
        self._initialized = False

        # Condition-change callbacks: condition_name → [callables]
        self._callbacks: dict = {
            "flat": [], "gentle_slope": [], "steep_slope": [], "vertical": []
        }
        self._last_condition = ""

    # ----- public API -----

    def register_callback(
        self, condition: str, callback: Callable[[SlopeReading], None]
    ):
        """Register a function to be called when a condition is first entered.

        Args:
            condition: One of "flat", "gentle_slope", "steep_slope", "vertical".
            callback:  Receives the SlopeReading that triggered the transition.
        """
        if condition in self._callbacks:
            self._callbacks[condition].append(callback)

    def add_sample(
        self,
        distance_cm: float,
        timestamp: Optional[float] = None,
    ) -> SlopeReading:
        """Add a new distance sample and return the updated slope reading.

        Args:
            distance_cm: Measured (bias-corrected) distance in centimetres.
            timestamp:   Epoch time in seconds; defaults to time.time().
        """
        ts = timestamp if timestamp is not None else time.time()
        self._timestamps.append(ts)
        self._distances.append(distance_cm)

        n = len(self._timestamps)

        # --- Need at least 2 points for a meaningful regression ---
        if n < 2:
            return SlopeReading(
                timestamp=ts,
                distance_cm=distance_cm,
                slope_deg=0.0,
                slope_rise_run=0.0,
                smoothed_slope_deg=0.0,
                confidence=0.0,
                condition="flat",
            )

        # --- Linear regression: distance vs time ---
        ts_list = list(self._timestamps)
        dist_list = list(self._distances)

        # Shift timestamps so the window starts at t=0 (numerical stability)
        t0 = ts_list[0]
        xs = [t - t0 for t in ts_list]
        ys = dist_list

        slope_cm_s, _intercept, residual_rms = linear_regression(xs, ys)

        # --- Convert slope (cm/s) to angle (degrees) ---
        # θ = arctan(Δdistance / Δtime) converted from radians to degrees.
        # Positive → distance increasing (receding); negative → approaching.
        measured_slope_deg = math.degrees(math.atan(slope_cm_s))
        
        # --- Apply mounting angle correction ---
        # If sensor is mounted at an angle (e.g. 45° downward), subtract that offset
        # from the measured slope to get the true ground slope.
        slope_deg = measured_slope_deg - self.mounting_angle_deg

        # --- Exponential Moving Average (EMA) smoothing ---
        # EMA_new = α · current + (1 − α) · previous
        if not self._initialized:
            self._smoothed_slope_deg = slope_deg
            self._initialized = True
        else:
            self._smoothed_slope_deg = (
                self.ema_alpha * slope_deg
                + (1.0 - self.ema_alpha) * self._smoothed_slope_deg
            )

        # --- Confidence metric [0, 1] ---
        # fill_fraction: how much of the window is populated (ramps up at start)
        fill_frac = n / self.window_size
        # residual_factor: how well a line fits the data (1 = perfect fit)
        residual_factor = max(0.0, 1.0 - residual_rms / self.MAX_RESIDUAL_CM)
        confidence = round(fill_frac * residual_factor, 3)

        # --- Condition classification ---
        abs_slope = abs(slope_deg)
        variance = sample_variance(dist_list)

        if abs_slope > self.STEEP_MAX_DEG or residual_rms > self.MAX_RESIDUAL_CM:
            condition = "vertical"
        elif abs_slope >= self.GENTLE_MAX_DEG:
            condition = "steep_slope"
        elif abs_slope >= self.FLAT_MAX_DEG:
            condition = "gentle_slope"
        else:
            # Only "flat" when variance is also low; noisy ≈ gentle
            condition = "flat" if variance < self.FLAT_MAX_VARIANCE else "gentle_slope"

        reading = SlopeReading(
            timestamp=ts,
            distance_cm=round(distance_cm, 2),
            slope_deg=round(slope_deg, 2),
            slope_rise_run=round(slope_cm_s, 4),
            smoothed_slope_deg=round(self._smoothed_slope_deg, 2),
            confidence=confidence,
            condition=condition,
        )

        # --- Fire callbacks on condition *transition* ---
        if condition != self._last_condition:
            self._last_condition = condition
            for cb in self._callbacks.get(condition, []):
                try:
                    cb(reading)
                except Exception as e:
                    print(f"[callback error] {e}", file=sys.stderr)

        return reading

    def reset(self):
        """Clear the sliding window and reset all internal state."""
        self._timestamps.clear()
        self._distances.clear()
        self._smoothed_slope_deg = 0.0
        self._initialized = False
        self._last_condition = ""


# ---------------------------------------------------------------------------
# Calibrator — sensor bias and noise measurement
# ---------------------------------------------------------------------------

class Calibrator:
    """Calibration routine to measure sensor bias and noise.

    Procedure:
      1. Place the sensor facing a flat surface at a known distance.
      2. Collect M distance samples.
      3. Compute mean and standard deviation.
      4. bias = mean − expected_distance
      5. Subtract this bias from future readings for correction.
    """

    def __init__(
        self,
        reader,
        sample_count: int = 50,
        sample_interval: float = 0.05,
    ):
        """
        Args:
            reader:          An UltrasonicReader or MockUltrasonicReader instance.
            sample_count:    Number of samples to collect (default 50).
            sample_interval: Seconds between samples (default 0.05 = 20 Hz).
        """
        self.reader = reader
        self.sample_count = sample_count
        self.sample_interval = sample_interval

    def run(self, expected_distance_cm: float = 0.0) -> CalibrationResult:
        """Collect samples and compute calibration parameters.

        Args:
            expected_distance_cm: True distance to the target surface (cm).
                If 0, only mean/stddev are computed (bias stays 0).

        Returns:
            CalibrationResult with mean, stddev, bias, and valid sample count.
        """
        samples: List[float] = []

        for _ in range(self.sample_count):
            d = self.reader.read_distance_cm()
            if d is not None:
                samples.append(d)
            time.sleep(self.sample_interval)

        if not samples:
            return CalibrationResult(0.0, 0.0, 0.0, 0)

        n = len(samples)
        mean = sum(samples) / n
        variance = sum((s - mean) ** 2 for s in samples) / max(n - 1, 1)
        stddev = math.sqrt(variance)

        bias = (mean - expected_distance_cm) if expected_distance_cm > 0 else 0.0

        return CalibrationResult(
            mean_cm=round(mean, 3),
            stddev_cm=round(stddev, 3),
            bias_cm=round(bias, 3),
            samples=n,
        )


# ---------------------------------------------------------------------------
# CLI — command-line interface with timestamped CSV output
# ---------------------------------------------------------------------------

class CLI:
    """Live CLI that prints one CSV line per sample until Ctrl+C."""

    def __init__(
        self,
        reader,
        estimator: SlopeEstimator,
        rate_hz: float = 10.0,
    ):
        self.reader = reader
        self.estimator = estimator
        self.rate_hz = rate_hz
        self._running = False

    def run(self):
        """Print timestamped CSV lines at the configured rate.

        Output format:
            timestamp, distance_cm, slope_deg, smoothed_slope_deg, confidence, condition
        """
        interval = 1.0 / self.rate_hz
        self._running = True

        # CSV header
        print("timestamp,distance_cm,slope_deg,smoothed_slope_deg,confidence,condition")

        try:
            while self._running:
                d = self.reader.read_distance_cm()
                if d is not None:
                    reading = self.estimator.add_sample(d)
                    print(
                        f"{reading.timestamp:.3f},"
                        f"{reading.distance_cm:.2f},"
                        f"{reading.slope_deg:.2f},"
                        f"{reading.smoothed_slope_deg:.2f},"
                        f"{reading.confidence:.3f},"
                        f"{reading.condition}"
                    )
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False

    def stop(self):
        """Signal the run loop to exit."""
        self._running = False


# ---------------------------------------------------------------------------
# Simulation / unit-test harness
# ---------------------------------------------------------------------------

class SimulationTestHarness:
    """Feeds synthetic distance sequences through the SlopeEstimator and
    verifies that slope conditions are detected correctly.

    Scenarios tested:
      - constant distance  → flat
      - slow linear ramp   → gentle_slope
      - fast linear ramp   → steep_slope
      - extreme ramp       → vertical
      - noisy flat surface  → still flat / gentle
      - exponential curve   → produces valid readings
      - regression math     → known slope & intercept
      - calibrator          → correct mean / bias
      - EMA smoothing       → reduces variance
      - callbacks           → fire on transitions
    """

    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.results: List[str] = []

    def _assert(self, name: str, condition: bool, detail: str = ""):
        """Record a pass/fail result for one assertion."""
        if condition:
            self.passed += 1
            self.results.append(f"  PASS: {name}")
        else:
            self.failed += 1
            self.results.append(f"  FAIL: {name} -- {detail}")

    # ----- runner -----

    def run_all(self) -> bool:
        """Execute every test scenario. Returns True if all pass."""
        print("=" * 60)
        print("Slope Profiler — Simulation Test Suite")
        print("=" * 60)

        self._test_flat_surface()
        self._test_gentle_slope()
        self._test_steep_slope()
        self._test_vertical_surface()
        self._test_noisy_flat()
        self._test_exponential_approach()
        self._test_linear_regression_math()
        self._test_calibrator()
        self._test_ema_smoothing()
        self._test_callbacks()

        print()
        for r in self.results:
            print(r)
        print()
        total = self.passed + self.failed
        print(f"Results: {self.passed}/{total} passed, {self.failed} failed")
        print("=" * 60)
        return self.failed == 0

    # ----- individual tests -----

    def _test_flat_surface(self):
        """Constant distance (100 cm) → slope ≈ 0°, condition = flat."""
        print("\n[Test] Flat surface (constant distance)")
        est = SlopeEstimator(window_size=10)
        t = 0.0
        last = None
        for _ in range(15):
            last = est.add_sample(100.0, timestamp=t)
            t += 0.1

        self._assert(
            "flat: slope < 5 deg",
            abs(last.slope_deg) < 5.0,
            f"slope={last.slope_deg}",
        )
        self._assert(
            "flat: condition",
            last.condition == "flat",
            f"condition={last.condition}",
        )

    def _test_gentle_slope(self):
        """Slow linear decrease → 5° <= |slope| < 15°."""
        print("\n[Test] Gentle slope (slow linear decrease)")
        est = SlopeEstimator(window_size=20)
        t = 0.0
        last = None
        # tan(10°) ≈ 0.1763; at 0.1 s steps: 0.01763 cm/step → 0.1763 cm/s
        for i in range(25):
            d = 100.0 - i * 0.01763
            last = est.add_sample(d, timestamp=t)
            t += 0.1

        self._assert(
            "gentle: 5 <= |slope| < 15",
            5.0 <= abs(last.slope_deg) < 15.0,
            f"slope={last.slope_deg}",
        )
        self._assert(
            "gentle: condition",
            last.condition == "gentle_slope",
            f"condition={last.condition}",
        )

    def _test_steep_slope(self):
        """Fast linear decrease → |slope| >= 15°."""
        print("\n[Test] Steep slope (fast linear decrease)")
        est = SlopeEstimator(window_size=20)
        t = 0.0
        last = None
        # tan(30°) ≈ 0.5774; at 0.1 s steps: 0.05774 cm/step → 0.5774 cm/s
        for i in range(25):
            d = 200.0 - i * 0.05774
            last = est.add_sample(d, timestamp=t)
            t += 0.1

        self._assert(
            "steep: |slope| >= 15",
            abs(last.slope_deg) >= 15.0,
            f"slope={last.slope_deg}",
        )
        self._assert(
            "steep: condition",
            last.condition == "steep_slope",
            f"condition={last.condition}",
        )

    def _test_vertical_surface(self):
        """Extreme distance change → |slope| > 60°, condition = vertical."""
        print("\n[Test] Vertical surface (extreme distance change)")
        est = SlopeEstimator(window_size=10)
        t = 0.0
        last = None
        # tan(70°) ≈ 2.747; at 0.1 s steps: 0.2747 cm/step → 2.747 cm/s
        for i in range(15):
            d = 200.0 - i * 0.2747
            last = est.add_sample(d, timestamp=t)
            t += 0.1

        self._assert(
            "vertical: |slope| > 60",
            abs(last.slope_deg) > 60.0,
            f"slope={last.slope_deg}",
        )
        self._assert(
            "vertical: condition",
            last.condition == "vertical",
            f"condition={last.condition}",
        )

    def _test_noisy_flat(self):
        """Flat surface with deterministic oscillation → still flat or gentle."""
        print("\n[Test] Noisy flat surface")
        est = SlopeEstimator(window_size=20)
        t = 0.0
        last = None
        for i in range(25):
            noise = 0.3 * math.sin(i * 1.7)  # small ±0.3 cm ripple
            last = est.add_sample(100.0 + noise, timestamp=t)
            t += 0.1

        self._assert(
            "noisy flat: |slope| < 5",
            abs(last.slope_deg) < 5.0,
            f"slope={last.slope_deg}",
        )
        self._assert(
            "noisy flat: condition is flat or gentle",
            last.condition in ("flat", "gentle_slope"),
            f"condition={last.condition}",
        )

    def _test_exponential_approach(self):
        """Distance decreasing exponentially → produces a non-zero slope."""
        print("\n[Test] Exponential approach")
        est = SlopeEstimator(window_size=15)
        t = 0.0
        last = None
        for _ in range(20):
            # d(t) = 200 · e^(−0.1·t)
            d = 200.0 * math.exp(-0.1 * t)
            last = est.add_sample(d, timestamp=t)
            t += 0.1

        self._assert(
            "exponential: non-zero slope",
            last is not None and last.slope_deg != 0.0,
            f"slope={last.slope_deg if last else 'None'}",
        )

    def _test_linear_regression_math(self):
        """Verify regression against a perfect y = 2x + 1 dataset."""
        print("\n[Test] Linear regression math")
        xs = [0.0, 1.0, 2.0, 3.0, 4.0]
        ys = [1.0, 3.0, 5.0, 7.0, 9.0]
        slope, intercept, residual = linear_regression(xs, ys)

        self._assert(
            "regression slope=2",
            abs(slope - 2.0) < 0.001,
            f"slope={slope}",
        )
        self._assert(
            "regression intercept=1",
            abs(intercept - 1.0) < 0.001,
            f"intercept={intercept}",
        )
        self._assert(
            "regression residual~0",
            residual < 0.001,
            f"residual={residual}",
        )

    def _test_calibrator(self):
        """Calibrator with a mock reader at known ~50.2 cm mean."""
        print("\n[Test] Calibrator")
        mock = MockUltrasonicReader([50.2, 50.1, 50.3, 50.0, 50.4] * 10)
        cal = Calibrator(mock, sample_count=50, sample_interval=0.0)
        result = cal.run(expected_distance_cm=50.0)

        self._assert(
            "calibrator: mean ~ 50.2",
            abs(result.mean_cm - 50.2) < 0.5,
            f"mean={result.mean_cm}",
        )
        self._assert(
            "calibrator: bias ~ 0.2",
            abs(result.bias_cm - 0.2) < 0.5,
            f"bias={result.bias_cm}",
        )
        self._assert(
            "calibrator: stddev small",
            result.stddev_cm < 1.0,
            f"stddev={result.stddev_cm}",
        )

    def _test_ema_smoothing(self):
        """EMA smoothing damps step changes in slope."""
        print("\n[Test] EMA smoothing")
        est = SlopeEstimator(window_size=10, ema_alpha=0.2)
        t = 0.0
        readings = []

        # Phase 1: flat (10 samples) → establishes a baseline slope near 0
        for _ in range(12):
            r = est.add_sample(100.0, timestamp=t)
            t += 0.1

        # Phase 2: sudden steep ramp → raw slope jumps, EMA lags behind
        for i in range(15):
            d = 100.0 - i * 0.15  # roughly tan(~56°) = 1.5 cm/s
            r = est.add_sample(d, timestamp=t)
            readings.append(r)
            t += 0.1

        # After the step change, smoothed slope should lag behind raw slope
        # (i.e. |smoothed| < |raw| for the first few ramp samples)
        lagged_count = sum(
            1 for r in readings[:8]
            if abs(r.smoothed_slope_deg) < abs(r.slope_deg)
        )

        self._assert(
            "EMA lags behind raw after step change",
            lagged_count >= 4,
            f"lagged_count={lagged_count}/8",
        )

    def _test_callbacks(self):
        """Callbacks fire when condition transitions to steep_slope."""
        print("\n[Test] Condition callbacks")
        est = SlopeEstimator(window_size=10)
        fired: List[str] = []
        est.register_callback(
            "steep_slope", lambda r: fired.append(r.condition)
        )

        t = 0.0
        # Start with flat readings
        for _ in range(10):
            est.add_sample(100.0, timestamp=t)
            t += 0.1
        # Transition to steep slope: tan(31°) ≈ 0.6 → 0.06 cm per 0.1 s
        for i in range(10):
            d = 100.0 - i * 0.06
            est.add_sample(d, timestamp=t)
            t += 0.1

        self._assert(
            "callback fired for steep_slope",
            "steep_slope" in fired,
            f"fired={fired}",
        )


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Ultrasonic Slope Profiler for Raspberry Pi",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--cli", action="store_true",
        help="Run CLI mode with live CSV output",
    )
    parser.add_argument(
        "--calibrate", action="store_true",
        help="Run calibration routine",
    )
    parser.add_argument(
        "--test", "--simulate", action="store_true",
        help="Run simulation / unit tests",
    )
    parser.add_argument(
        "--rate", type=float, default=10.0,
        help="Sampling rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--window", type=int, default=20,
        help="Sliding window size (default: 20)",
    )
    parser.add_argument(
        "--alpha", type=float, default=0.3,
        help="EMA smoothing alpha 0–1 (default: 0.3)",
    )
    parser.add_argument(
        "--trig", type=int, default=23,
        help="GPIO pin for TRIG (default: 23)",
    )
    parser.add_argument(
        "--echo", type=int, default=24,
        help="GPIO pin for ECHO (default: 24)",
    )
    parser.add_argument(
        "--cal-samples", type=int, default=50,
        help="Number of calibration samples (default: 50)",
    )
    parser.add_argument(
        "--cal-distance", type=float, default=0.0,
        help="Known distance for calibration in cm (default: 0 = auto)",
    )
    parser.add_argument(
        "--mounting-angle", type=float, default=0.0,
        help="Sensor mounting angle in degrees (e.g. 45 for downward tilt; default: 0)",
    )
    args = parser.parse_args()

    # ---- Simulation / test mode ----
    if args.test:
        harness = SimulationTestHarness()
        success = harness.run_all()
        sys.exit(0 if success else 1)

    # ---- Create a hardware or mock reader ----
    if _ON_PI:
        reader = UltrasonicReader(trig_pin=args.trig, echo_pin=args.echo)
    else:
        print(
            "[INFO] RPi.GPIO not available — using mock reader for desktop demo",
            file=sys.stderr,
        )
        # Simulate a slow approach: 200 cm → 0 cm in 0.5 cm steps
        distances = [200.0 - i * 0.5 for i in range(400)]
        reader = MockUltrasonicReader(distances)

    # ---- Calibration mode ----
    if args.calibrate:
        print(f"Calibrating with {args.cal_samples} samples ...")
        cal = Calibrator(reader, sample_count=args.cal_samples)
        result = cal.run(expected_distance_cm=args.cal_distance)
        print(f"  Mean:          {result.mean_cm:.3f} cm")
        print(f"  Std deviation: {result.stddev_cm:.3f} cm")
        print(f"  Bias:          {result.bias_cm:.3f} cm")
        print(f"  Valid samples: {result.samples}/{args.cal_samples}")
        if hasattr(reader, "bias_cm"):
            reader.bias_cm = result.bias_cm
            print(f"  Bias correction applied: {result.bias_cm:.3f} cm")
        reader.cleanup()
        return

    # ---- CLI mode (default) ----
    estimator = SlopeEstimator(
        window_size=args.window, 
        ema_alpha=args.alpha, 
        mounting_angle_deg=args.mounting_angle
    )

    # Register demo callbacks that print condition transitions to stderr
    def on_condition_change(reading: SlopeReading):
        print(
            f"[EVENT] Condition -> {reading.condition} "
            f"(slope={reading.slope_deg:.1f} deg)",
            file=sys.stderr,
        )

    for cond in ("flat", "gentle_slope", "steep_slope", "vertical"):
        estimator.register_callback(cond, on_condition_change)

    cli = CLI(reader, estimator, rate_hz=args.rate)
    try:
        cli.run()
    finally:
        reader.cleanup()


if __name__ == "__main__":
    main()
