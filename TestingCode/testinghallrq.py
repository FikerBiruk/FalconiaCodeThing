import smbus
import time

bus = smbus.SMBus(1)
address = 0x48
time.sleep(0.5)

while True:
    bus.write_byte(address, 0x40)
    value = bus.read_byte(address)

    print(value)

    time.sleep(0.2)