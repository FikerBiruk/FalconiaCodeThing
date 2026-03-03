import atexit
import time

from adafruit_motorkit import MotorKit


def main() -> None:
    kit = MotorKit(address=0x60)

    motors = [kit.motor1, kit.motor2, kit.motor3, kit.motor4]

    def release_all() -> None:
        for motor in motors:
            motor.throttle = None

    atexit.register(release_all)

    print("MotorKit port test starting (addr=0x60)")
    print("Each port M1..M4 runs FORWARD at full speed for 2 seconds.")

    for i, motor in enumerate(motors, start=1):
        print(f"Testing M{i} FORWARD @1.0 for 2s...")
        motor.throttle = 1.0
        time.sleep(2)
        motor.throttle = None
        print(f"M{i} released.")
        time.sleep(1)

    print("Port test complete.")


if __name__ == "__main__":
    main()
