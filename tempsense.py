import glob

device = glob.glob('/sys/b1/w1/devices/28-*')[0] + '/w1_slave'
raw = open(device).read()
temp = raw.split('t=')[-1]
print(float(temp)/1000)
