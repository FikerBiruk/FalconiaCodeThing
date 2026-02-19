import smbus2
import time

# PCF8591 configuration
PCF8591_ADDRESS = 0x48  # Default I2C address
HALL_CHANNEL = 0        # AIN0 (change to 1, 2, or 3 if using different channel)

# Threshold for magnet detection (adjust based on your sensor)
MAGNET_THRESHOLD = 150  # 0-255 range (8-bit ADC)

bus = smbus2.SMBus(1)  # Use I2C bus 1

def read_analog(channel):
    """Read analog value from PCF8591 channel (0-3)."""
    # Control byte: 0x40 + channel number
    control_byte = 0x40 | channel
    
    # First read is previous value, second read is current
    bus.write_byte(PCF8591_ADDRESS, control_byte)
    bus.read_byte(PCF8591_ADDRESS)  # Discard first read
    value = bus.read_byte(PCF8591_ADDRESS)  # Get actual value
    
    return value

try:
    print("Analog Hall Sensor Test")
    print(f"Reading from PCF8591 channel {HALL_CHANNEL}")
    print("Press Ctrl+C to exit\n")
    
    while True:
        # Read analog value (0-255)
        value = read_analog(HALL_CHANNEL)
        
        # Convert to voltage (assuming 3.3V reference)
        voltage = (value / 255.0) * 3.3
        
        # Check if magnet detected
        if value > MAGNET_THRESHOLD:
            status = "MAGNET DETECTED"
        else:
            status = "No magnet"
        
        print(f"Value: {value:3d} | Voltage: {voltage:.2f}V | {status}")
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nTest stopped")
    bus.close()
