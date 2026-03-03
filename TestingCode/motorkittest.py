import time
import atexit
from adafruit_motorkit import MotorKit

# Initialize MotorKit at default I2C address
kit = MotorKit(address=0x60)

# Clean shutdown: release all motors on exit
def turn_off_motors():
    kit.motor1.throttle = None
    kit.motor2.throttle = None

atexit.register(turn_off_motors)

# Get DC motors on ports M1 and M2
motor1 = kit.motor1
motor2 = kit.motor2

# Set speed to 50% forward (0.5 on a -1.0 to 1.0 scale)
motor1.throttle = 0.5
motor2.throttle = 0.5

# Run forward for 10 seconds
time.sleep(10)

# Reverse direction at 50% for 10 seconds
motor1.throttle = -0.5
motor2.throttle = -0.5
time.sleep(10)

# Stop both motors
motor1.throttle = None
motor2.throttle = None
