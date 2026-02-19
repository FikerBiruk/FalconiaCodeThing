"""Simple camera capture script for Raspberry Pi.

Captures images from the default camera when spacebar is pressed.
No preview window - uses terminal for controls.
"""
import cv2
import os
from pynput import keyboard

# Configuration
OUTPUT_DIR = "images"
RESOLUTION = (1280, 720)
image_counter = 0

# Create output directory if it doesn't exist
if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)
    print(f"Created directory: {OUTPUT_DIR}")

# Initialize camera
print("Initializing camera...")
cap = cv2.VideoCapture(0)

# Set resolution (will use closest supported resolution)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])

if not cap.isOpened():
    print("Error: Unable to open camera")
    exit(1)

# Get actual resolution
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera initialized at {actual_width}x{actual_height}")
print("\nControls:")
print("  SPACEBAR - Capture image")
print("  Q        - Quit")
print("\nReady! Press SPACEBAR to capture images...\n")


def on_press(key):
    """Handle keyboard input."""
    global image_counter
    
    try:
        # Check for spacebar
        if key == keyboard.Key.space:
            # Read frame from camera
            ret, frame = cap.read()
            
            if ret:
                # Generate filename with zero-padded number
                filename = f"{OUTPUT_DIR}/frame_{image_counter:04d}.jpg"
                
                # Save image
                cv2.imwrite(filename, frame)
                print(f"✓ Captured: {filename}")
                image_counter += 1
            else:
                print("✗ Failed to capture frame")
        
        # Check for 'q' key
        elif hasattr(key, 'char') and key.char == 'q':
            print("\nExiting...")
            return False  # Stop listener
            
    except AttributeError:
        pass  # Special key not handled


# Set up keyboard listener
try:
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()
finally:
    # Clean up
    cap.release()
    print(f"Camera released. Total images captured: {image_counter}")
