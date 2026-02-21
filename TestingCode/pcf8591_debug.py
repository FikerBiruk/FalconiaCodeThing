#!/usr/bin/env python3
"""
PCF8591 ADC Debugging Script for Raspberry Pi
Reads analog data from PCF8591 ADC at I2C address 0x48
Uses smbus2 for I2C communication
"""

import smbus2
import time
import sys

# PCF8591 Configuration
PCF8591_ADDR = 0x48
PCF8591_CMD_CHANNEL0 = 0x00
PCF8591_CMD_CHANNEL1 = 0x01
PCF8591_CMD_CHANNEL2 = 0x02
PCF8591_CMD_CHANNEL3 = 0x03

READ_INTERVAL = 0.1  # 10 Hz sampling


def init_bus(bus_num=1):
    """Initialize I2C bus"""
    try:
        bus = smbus2.SMBus(bus_num)
        return bus
    except Exception as e:
        print(f"Error initializing I2C bus: {e}")
        sys.exit(1)


def read_channel(bus, channel):
    """
    Read analog value from specified channel (0-3)
    Implements PCF8591 dummy-read sequence:
    - First read returns previous conversion result
    - Send command for desired channel
    - Second read returns current conversion result
    """
    if channel < 0 or channel > 3:
        print(f"Invalid channel {channel}. Must be 0-3")
        return None
    
    try:
        # Dummy read (discard result, selects channel for next read)
        bus.read_byte(PCF8591_ADDR)
        
        # Actual read from selected channel
        value = bus.read_byte(PCF8591_ADDR)
        return value
    except Exception as e:
        print(f"Error reading channel {channel}: {e}")
        return None


def read_channel0_continuous(bus, duration=10, verbose=True):
    """
    Read analog channel 0 continuously at 10 Hz
    
    Args:
        bus: SMBus instance
        duration: How long to read in seconds
        verbose: Print values to console
    """
    print(f"\nReading Channel 0 for {duration} seconds...")
    start_time = time.time()
    count = 0
    
    while time.time() - start_time < duration:
        value = read_channel(bus, PCF8591_CMD_CHANNEL0)
        if value is not None:
            if verbose:
                print(f"Channel 0: {value:3d} (Raw: 0x{value:02x})")
            count += 1
        time.sleep(READ_INTERVAL)
    
    print(f"Read {count} samples in {time.time() - start_time:.1f} seconds")
    return count


def test_all_channels(bus, samples_per_channel=5):
    """
    Test all four analog channels
    
    Args:
        bus: SMBus instance
        samples_per_channel: Number of samples to take per channel
    """
    print("\nTesting All Channels...")
    print("-" * 40)
    
    channels = [
        (PCF8591_CMD_CHANNEL0, "Channel 0 (AIN0)"),
        (PCF8591_CMD_CHANNEL1, "Channel 1 (AIN1)"),
        (PCF8591_CMD_CHANNEL2, "Channel 2 (AIN2)"),
        (PCF8591_CMD_CHANNEL3, "Channel 3 (AIN3)"),
    ]
    
    for cmd, label in channels:
        values = []
        for i in range(samples_per_channel):
            value = read_channel(bus, cmd)
            if value is not None:
                values.append(value)
            time.sleep(0.05)
        
        if values:
            avg = sum(values) / len(values)
            min_val = min(values)
            max_val = max(values)
            print(f"{label}: min={min_val:3d}, max={max_val:3d}, avg={avg:6.1f}")
        else:
            print(f"{label}: ERROR - No valid readings")
    
    print("-" * 40)


def detect_wiring(bus, samples=10, threshold_low=10, threshold_high=245):
    """
    Detect if PCF8591 is wired correctly
    - AIN0 tied to GND should read ~0
    - AIN0 tied to 3.3V should read ~255
    
    Args:
        bus: SMBus instance
        samples: Number of samples per test
        threshold_low: Max value to consider as "GND" (default 10)
        threshold_high: Min value to consider as "3.3V" (default 245)
    
    Returns:
        dict with test results
    """
    print("\n" + "="*50)
    print("PCF8591 Wiring Detection")
    print("="*50)
    print("\nTest 1: Connect AIN0 to GND, press ENTER to continue...")
    input()
    
    values_gnd = []
    for i in range(samples):
        value = read_channel(bus, PCF8591_CMD_CHANNEL0)
        if value is not None:
            values_gnd.append(value)
        time.sleep(0.1)
    
    avg_gnd = sum(values_gnd) / len(values_gnd) if values_gnd else None
    gnd_ok = avg_gnd is not None and avg_gnd < threshold_low
    
    print(f"GND Test: min={min(values_gnd)}, max={max(values_gnd)}, avg={avg_gnd:.1f}")
    print(f"Result: {'PASS' if gnd_ok else 'FAIL'} (threshold < {threshold_low})")
    
    print("\nTest 2: Connect AIN0 to 3.3V, press ENTER to continue...")
    input()
    
    values_3v3 = []
    for i in range(samples):
        value = read_channel(bus, PCF8591_CMD_CHANNEL0)
        if value is not None:
            values_3v3.append(value)
        time.sleep(0.1)
    
    avg_3v3 = sum(values_3v3) / len(values_3v3) if values_3v3 else None
    v3v3_ok = avg_3v3 is not None and avg_3v3 > threshold_high
    
    print(f"3.3V Test: min={min(values_3v3)}, max={max(values_3v3)}, avg={avg_3v3:.1f}")
    print(f"Result: {'PASS' if v3v3_ok else 'FAIL'} (threshold > {threshold_high})")
    
    print("\n" + "="*50)
    if gnd_ok and v3v3_ok:
        print("✓ PCF8591 wiring appears CORRECT")
    else:
        print("✗ PCF8591 wiring has issues:")
        if not gnd_ok:
            print("  - GND test failed (check AIN0 to GND connection)")
        if not v3v3_ok:
            print("  - 3.3V test failed (check AIN0 to 3.3V connection)")
    print("="*50 + "\n")
    
    return {
        "gnd_test": gnd_ok,
        "v3v3_test": v3v3_ok,
        "gnd_avg": avg_gnd,
        "v3v3_avg": avg_3v3,
    }


def main():
    """Main entry point"""
    print("PCF8591 ADC Debugging Script")
    print("I2C Address: 0x48")
    print("-" * 40)
    
    try:
        bus = init_bus(1)
        print("I2C bus initialized successfully")
    except Exception as e:
        print(f"Failed to initialize I2C bus: {e}")
        sys.exit(1)
    
    try:
        while True:
            print("\nOptions:")
            print("1. Read Channel 0 (10 seconds)")
            print("2. Test All Channels")
            print("3. Run Wiring Detection Test")
            print("4. Quick Channel 0 Read (30 samples)")
            print("5. Exit")
            
            choice = input("\nSelect option (1-5): ").strip()
            
            if choice == "1":
                read_channel0_continuous(bus, duration=10)
            
            elif choice == "2":
                test_all_channels(bus, samples_per_channel=5)
            
            elif choice == "3":
                detect_wiring(bus, samples=10)
            
            elif choice == "4":
                print("\nQuick Channel 0 Read (3 seconds at 10 Hz)...")
                read_channel0_continuous(bus, duration=3)
            
            elif choice == "5":
                print("Exiting...")
                bus.close()
                break
            
            else:
                print("Invalid option. Try again.")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        bus.close()
    except Exception as e:
        print(f"Error: {e}")
        bus.close()
        sys.exit(1)


if __name__ == "__main__":
    main()
