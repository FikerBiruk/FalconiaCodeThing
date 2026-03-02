import time
import atexit
from Adafruit_MotorHAT import Adafruit_MotorHAT

# Initialize MotorHAT at default I2C address
mh = Adafruit_MotorHAT(addr=0x60)

# Clean shutdown: release all motors on exit
def turn_off_motors():
    mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turn_off_motors)

# Get DC motor on port M1
motor = mh.getMotor(1)

# Set speed to 50% (128 out of 255)
motor.setSpeed(128)

# Run forward for 3 seconds
motor.run(Adafruit_MotorHAT.FORWARD)
time.sleep(3)

# Stop the motor
motor.run(Adafruit_MotorHAT.RELEASE)
