import time
import atexit
from adafruit_motorkit import MotorKit

# Initialize MotorKit at default I2C address
kit = MotorKit(address=0x60)

# Clean shutdown: release all motors on exit
def turn_off_motors():
    kit.motor1.throttle = None

atexit.register(turn_off_motors)

# Get DC motor on port M1
motor = kit.motor1

# Set speed to 50% (0.5 on a -1.0 to 1.0 scale)
motor.throttle = 0.5

# Run forward for 3 seconds
time.sleep(3)

# Stop the motor
motor.throttle = None
