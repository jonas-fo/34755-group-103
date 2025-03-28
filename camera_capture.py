import threading
import time as t
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
from scam import cam  # Import the camera class
from uservice import service
import os 

# Create directory if it doesn't exist
save_path = "mqtt_python/group_103/captured_images"
if not os.path.exists(save_path):
    os.makedirs(save_path)

# To control the capturing process with a threading lock to prevent multiple simultaneous captures
capture_lock = threading.Lock()

def capture_image(save=True):
    # Check if the lock is available (no other capture is in progress)
    with capture_lock:
        ok, img, imgTime = cam.getImage()  # Capture image from the camera
        if not ok:  # Check if image was captured successfully
            if cam.imageFailCnt < 5:
                print("% Failed to get image.")
        else:
            if save:
                # Saving the image with a timestamped filename
                img_name = os.path.join(save_path, f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg")
                cv.imwrite(img_name, img)
                print(f"% Saved image {img_name}")

def capture_images():
    while True:
        user_input = input("Press Enter to capture an image or type 'exit' to quit: ")
        if user_input.lower() == 'exit':
            break
        else:
            capture_image()  # Trigger image capture when Enter is pressed


if __name__ == "__main__":
    setproctitle("mqtt-client")
    print("% Starting")
    
    # Setup service (assuming you have already set up the service in your script)
    service.setup('localhost')  # localhost, or whatever server address you need

    if service.connected:
        # Run the image capture loop in a separate thread
        image_capture_thread = threading.Thread(target=capture_images, daemon=True)
        image_capture_thread.start()

        print("System initialized. Running normally...")

        # Run the main loop which will listen for the exit command only
        capture_images()

    service.terminate()
    print("% Main Terminated")
