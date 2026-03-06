import atexit
import curses
import time

import board
from adafruit_motorkit import MotorKit


I2C_ADDRESS = 0x60
LEFT_MOTOR_ID = 1
RIGHT_MOTOR_ID = 2
HOLD_TIMEOUT_SECONDS = 0.60
LOOP_DELAY_SECONDS = 0.02
ACCEL_RATE_PER_SECOND = 7.5
DECEL_RATE_PER_SECOND = 12.0


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def move_toward(current: float, target: float, max_delta: float) -> float:
    if target > current:
        return min(current + max_delta, target)
    if target < current:
        return max(current - max_delta, target)
    return current


def apply_motor_command(motor, command: float) -> None:
    motor.throttle = command


def get_motor_by_id(kit: MotorKit, motor_id: int):
    motor_map = {
        1: kit.motor1,
        2: kit.motor2,
        3: kit.motor3,
        4: kit.motor4,
    }
    if motor_id not in motor_map:
        raise ValueError(f"Invalid motor ID: {motor_id}. Expected 1-4.")
    return motor_map[motor_id]


def run_keyboard_control(stdscr) -> None:
    kit = MotorKit(i2c=board.I2C(), address=I2C_ADDRESS)
    left_motor = get_motor_by_id(kit, LEFT_MOTOR_ID)
    right_motor = get_motor_by_id(kit, RIGHT_MOTOR_ID)

    def release_motors() -> None:
        left_motor.throttle = None
        right_motor.throttle = None

    atexit.register(release_motors)

    stdscr.nodelay(True)
    stdscr.keypad(False)
    curses.curs_set(0)

    key_last_seen = {"w": 0.0, "a": 0.0, "s": 0.0, "d": 0.0}
    left_output = 0.0
    right_output = 0.0

    while True:
        now = time.monotonic()

        while True:
            key_code = stdscr.getch()
            if key_code == -1:
                break

            if key_code in (ord("q"), ord("Q")):
                return

            if key_code < 0:
                continue

            pressed = chr(key_code).lower()
            if pressed in key_last_seen:
                key_last_seen[pressed] = now

        w_active = (now - key_last_seen["w"]) < HOLD_TIMEOUT_SECONDS
        s_active = (now - key_last_seen["s"]) < HOLD_TIMEOUT_SECONDS
        a_active = (now - key_last_seen["a"]) < HOLD_TIMEOUT_SECONDS
        d_active = (now - key_last_seen["d"]) < HOLD_TIMEOUT_SECONDS

        if w_active and not s_active:
            throttle = 1.0
        elif s_active and not w_active:
            throttle = -1.0
        else:
            throttle = 0.0

        if d_active and not a_active:
            steer = 1.0
        elif a_active and not d_active:
            steer = -1.0
        else:
            steer = 0.0

        left_target = clamp(throttle + steer, -1.0, 1.0)
        right_target = clamp(throttle - steer, -1.0, 1.0)

        left_delta_limit = DECEL_RATE_PER_SECOND * LOOP_DELAY_SECONDS if abs(left_target) < abs(left_output) else ACCEL_RATE_PER_SECOND * LOOP_DELAY_SECONDS
        right_delta_limit = DECEL_RATE_PER_SECOND * LOOP_DELAY_SECONDS if abs(right_target) < abs(right_output) else ACCEL_RATE_PER_SECOND * LOOP_DELAY_SECONDS

        left_output = move_toward(left_output, left_target, left_delta_limit)
        right_output = move_toward(right_output, right_target, right_delta_limit)

        apply_motor_command(left_motor, left_output)
        apply_motor_command(right_motor, right_output)

        stdscr.erase()
        stdscr.addstr(0, 0, "Rover Keyboard Control (SSH)")
        stdscr.addstr(1, 0, "Hold W/A/S/D to drive, Q to quit")
        stdscr.addstr(3, 0, f"W:{int(w_active)} A:{int(a_active)} S:{int(s_active)} D:{int(d_active)}")
        stdscr.addstr(4, 0, f"Left cmd : {left_output:+.2f} (target {left_target:+.2f})")
        stdscr.addstr(5, 0, f"Right cmd: {right_output:+.2f} (target {right_target:+.2f})")
        stdscr.addstr(7, 0, f"Motors: left=M{LEFT_MOTOR_ID}, right=M{RIGHT_MOTOR_ID}, addr=0x{I2C_ADDRESS:02X}")
        stdscr.refresh()

        time.sleep(LOOP_DELAY_SECONDS)


if __name__ == "__main__":
    try:
        curses.wrapper(run_keyboard_control)
    except KeyboardInterrupt:
        pass