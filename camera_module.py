from picamera2 import Picamera2

picam = Picamera2()
picam.configure(picam.create_video_configuration(main={"size": (320, 240)}))
picam.start()

def capture_frame():
    return picam.capture_array()
