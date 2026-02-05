import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

IR_PIN = 25
GPIO.setup(IR_PIN, GPIO.IN)

try:
    while True:
        state = GPIO.input(IR_PIN)

        if state == 0:
            print("Obstacle detected")
        else:
            print("Clear")

        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()