import cv2
import numpy as np
from scam import cam
from setproctitle import setproctitle
from uservice import service
import threading
import os
import math
import time

#to save image processed: 
# Create directory if it doesn't exist
save_path = "/home/local/svn/robobot/mqtt_python/group_103/ball_detect_img"
if not os.path.exists(save_path):
    os.makedirs(save_path)


# Load calibration data
calib_data = np.load("/home/local/svn/robobot/calibration_data.npz")
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["distortion_coeffs"]
capture_lock = threading.Lock()


def pixel_to_world(x, y, camera_matrix, radius):
    """Convert pixel coordinates (x, y) to real-world coordinates (X, Y)"""
    fx = camera_matrix[0, 0]  # Focal length in x
    fy = camera_matrix[1, 1]  # Focal length in y
    cx = camera_matrix[0, 2]  # Optical center x
    cy = camera_matrix[1, 2]  # Optical center y

    # Known real-world ball diameter (e.g., 0.043m for a golf ball)
    ball_d = 0.043  # meters

    # Estimate depth (Z) using the known ball size
    if radius > 0:
        Z = (ball_d * fy) / (2 * radius)  # radius is the pixel radius from process frame
    else:
        Z = 0  # Default if radius is too small
        
    # Convert pixel (x, y) to world coordinates (assuming Z = 0)
    X = (x - cx) * Z / fx
    Y = (y - cy) * Z / fy

    """Now change to robots coordinates"""
    #Intrinsic properties of camera on robot NOT SURE ABOUT THESE VALUES YET
    theta=11
    cxr=0.016 #camera lean in x 
    cyr=0
    czr=0.188 #camera_height
    
    #transformation from world to robot: 
    T = np.array([
        [np.cos(theta), 0, np.sin(theta), -cxr],
        [0, 1, 0, cyr],
        [-np.sin(theta), 0, np.cos(theta), -czr],
        [0, 0, 0, 1]
    ])
    
    # Frame change matrix
    F = np.array([
        [0, 0, 1, 0],
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 0, 1]
    ])
    
    # Camera coordinates in [m] (the camera to world coordinates)
    C = np.array([[X], [Y], [Z], [1]])
    
    # Compute robot coordinates
    R = np.dot(np.dot(T, F), C)
    print("Robot coordinates:\n", R)
    R=R.flatten()

    return X, Y, Z, R

def move_robot_to_target(Z,X, stop_distance=0.30):
    
    ###########Method 1 for arriving at ball ###################
    forward_speed=0.15 # m/s
    turn_speed=0.5 # rad/s
    distance=Z-stop_distance
    
    #use the X,Z for the turn rate for 1 sec:
    angle=math.atan2(X,Z)

    turn_time=abs(angle/turn_speed)
    if angle > 0:
        print("Turning right...")
        service.send(service.topicCmd + "ti/rc", "0 " + str(-turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0 0")
        print("Turned right.")
        
    elif angle < 0:
        print("Turning lefft...")
        service.send(service.topicCmd + "ti/rc", "0 "+str(turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0 0")
        print("Turned lefft.")
        
    else:
        print("No turning needed.")
        
    
    
     #find the distance to the target after turning (centering)
    if distance <= 0:
        print("Already close enough to the target. OR too close CHECK")
        return
    else:
        move_time=distance/forward_speed

        print(f"Moving forward for {move_time:.2f} seconds...")

        service.send(service.topicCmd + "ti/rc", str(forward_speed)+"0")
        time.sleep(move_time)
        service.send(service.topicCmd + "ti/rc", "0 0")
        print("Arrived at target.")
        return 
    
    
    



def process_frame(camera_matrix, dist_coeffs, obj):
    save=True
    """Captures, undistorts, and detects a golf ball."""
    with capture_lock:
        ok, frame, imgTime = cam.getImage()
        if not ok:
            print("% Failed to get image.")
            return None, None  # Return None for both

        # Undistort the frame
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix,)

        if obj == 0:
            # Convert to HSV and apply thresholding
            hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])  
            upper_orange = np.array([15, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
        elif obj == 1:
            hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
            lower_brown = np.array([0, 0, 0])  
            upper_brown = np.array([42, 145, 176])
            mask = cv2.inRange(hsv, lower_brown, upper_brown)
        elif obj == 2:
            lower_red1 = np.array([0, 150, 100])
            upper_red1 = np.array([10, 255, 255])

            lower_red2 = np.array([170, 150, 100])
            upper_red2 = np.array([180, 255, 255])

            # Create masks and combine them
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("No contours detected.")
            return undistorted_frame, None

        # Get the largest detected contour
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 10:
            x, y, radius = int(x), int(y), int(radius)

            # Convert to real-world coordinates
            X_world, Y_world, Z_world, Robot_coor = pixel_to_world(x, y, camera_matrix, radius)

            # Draw detected ball
            cv2.circle(undistorted_frame, (x, y), radius, (0, 255, 0), 2)
            
            if save:
                # Saving the image with a timestamped filename
                img_name = os.path.join(save_path, f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}.jpg")
                cv2.imwrite(img_name, undistorted_frame)
                print(f"% Saved image {img_name}")
            return undistorted_frame, (X_world, Y_world, Z_world, Robot_coor)
        
        else:
            print("Contour found, but radius too small.")
            return undistorted_frame, None

def test_loop():
    """ Runs the main loop: captures frames, processes them, and detects the golf ball. """
    
    while True:
        user_input = input("Press Enter to move to object, 1 to capture image or type 'exit' to quit: ")
        if user_input.lower() == 'exit':
            break
        elif user_input.lower() == '1':
            processed_frame, ball_position = process_frame(camera_matrix, dist_coeffs,1)
            print("hole at: ", ball_position)
            move_robot_to_target(ball_position[2],ball_position[0])
            print("Moving robot to target...")
        else:
            processed_frame, ball_position = process_frame(camera_matrix, dist_coeffs,0)
            #print(f"Ball detected at: {ball_position}")
            if ball_position:
                print(f"Ball detected at: {ball_position}, Robot Z-coordinates: {ball_position[3][2]}")
                move_robot_to_target(ball_position[2],ball_position[0])
                print("Moving robot to target...")
                
            else:
                print("No ball detected.")
                continue
        



if __name__ == "__main__":
    setproctitle("mqtt-client")
    print("% Starting")
    
    # Setup service (assuming you have already set up the service in your script)
    #service.setup('localhost')  # localhost, or whatever server address you need
    service.setup('10.197.216.254')
    if service.connected:
        # Run the image capture loop in a separate thread
        image_capture_thread = threading.Thread(target=test_loop, daemon=True)
        image_capture_thread.start()

        print("System initialized. Running normally...")

        # Run the main loop which will listen for the exit command only
        test_loop()

        
        
        
    service.terminate()
    print("% Main Terminated")