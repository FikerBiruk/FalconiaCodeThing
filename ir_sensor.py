import RPi.GPIO as GPIO

IR_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)

def is_obstacle():
    return GPIO.input(IR_PIN) == 0
