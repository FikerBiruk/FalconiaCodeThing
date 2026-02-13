import glob, time

device = glob.glob('/sys/bus/w1/devices/28-*')[0] + '/w1_slave'

while True:
with open(device, 'r') as f:
raw = f.read()
temp = raw.split('t=')[-1]
print(float(temp) / 1000)
time.sleep(1)
