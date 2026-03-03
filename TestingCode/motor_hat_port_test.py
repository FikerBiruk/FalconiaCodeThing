import atexit
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT


def main() -> None:
    mh = Adafruit_MotorHAT(addr=0x60)

    def release_all() -> None:
        for motor_id in range(1, 5):
            mh.getMotor(motor_id).run(Adafruit_MotorHAT.RELEASE)

    atexit.register(release_all)

    print("Motor HAT port test starting (addr=0x60)")
    print("Each port M1..M4 runs FORWARD at full speed for 2 seconds.")

    for motor_id in range(1, 5):
        motor = mh.getMotor(motor_id)
        print(f"Testing M{motor_id} FORWARD @255 for 2s...")
        motor.setSpeed(255)
        motor.run(Adafruit_MotorHAT.FORWARD)
        time.sleep(2)
        motor.run(Adafruit_MotorHAT.RELEASE)
        print(f"M{motor_id} released.")
        time.sleep(1)

    print("Port test complete.")


if __name__ == "__main__":
    main()