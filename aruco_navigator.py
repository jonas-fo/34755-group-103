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


def pixel_to_world(x, y, camera_matrix, radius, object_d):
    """Convert pixel coordinates (x, y) to real-world coordinates (X, Y)"""
    fx = camera_matrix[0, 0]  # Focal length in x
    fy = camera_matrix[1, 1]  # Focal length in y
    cx = camera_matrix[0, 2]  # Optical center x
    cy = camera_matrix[1, 2]  # Optical center y

    # Known real-world ball diameter (e.g., 0.043m for a golf ball)
    #object_d = 0.043  # meters I changed this and now put it in the function call

    # Estimate depth (Z) using the known ball size
    if radius > 0:
        Z = (object_d * fy) / (2 * radius)  # radius is the pixel radius from process frame
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


def process_frame(camera_matrix, dist_coeffs, object_d, save=True):
    """Captures, undistorts, and detects a golf ball."""
    with capture_lock:
        ok, frame, imgTime = cam.getImage()
        if not ok:
            print("% Failed to get image.")
            return None, None  # Return None for both

        # Undistort the frame
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix,)

        if object_d == 0.043:
            # === Ball detection (orange color) ===
            hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 100])  
            upper_orange = np.array([15, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        elif object_d == 0.052:
            # === Hole detection (lower half, edge/shape-based) ===
            height, width, _ = undistorted_frame.shape
            lower_half = undistorted_frame[height // 2:, :]

            gray = cv2.cvtColor(lower_half, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (7, 7), 0)
            edged = cv2.Canny(blurred, 50, 150)
            # Find contours
            contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Adjust coordinates back to full frame after cropping
            for c in contours:
                for pt in c:
                    pt[0][1] += height // 2

        else:
            print("Unsupported object diameter.")
            return undistorted_frame, None



        if not contours:
            print("No contours detected.")
            return undistorted_frame, None

        # Get the largest detected contour
        c = max(contours, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius > 5:
            x, y, radius = int(x), int(y), int(radius)

            # Convert to real-world coordinates
            X_world, Y_world, Z_world, Robot_coor = pixel_to_world(x, y, camera_matrix, radius, object_d)

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


def move_robot_to_target(Z,X, stop_distance=0.35):
    
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
    
    #if distance < stop_distance:
    #    #Reverse
    #    print("Too close to the target. Reversing...")
    #    reverse_speed = -0.15  # m/s
    #    reverse_time = abs(distance / reverse_speed)
    #    service.send(service.topicCmd + "ti/rc", str(reverse_speed)+"0")
    #    time.sleep(reverse_time)
    #    service.send(service.topicCmd + "ti/rc", "0 0")
    #    print("Reversed away from target.")
    #    return
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
    
def turn(angle, turn_speed=0.5): # 90 degrees is 1.57 radians and -90 degrees is -1.57 radians i think 
    
    turn_time = abs(angle / turn_speed)
    if angle > 0:
        print("Turning right...")
        service.send(service.topicCmd + "ti/rc", "0 " + str(-turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0 0")
        print("Turned right.")
    elif angle < 0:
        print("Turning left...")
        service.send(service.topicCmd + "ti/rc", "0 " + str(turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0 0")
        print("Turned left.")
    else:
        print("No turning needed.")
    
def test_loop(object_d): #### 0.043 is for ball and 0.052 for hole
    """ Runs the main loop: captures frames, processes them, and detects the golf ball. """
    
    while True:
        user_input = input("Press Enter to capture an image or type 'exit' to quit: ")
        if user_input.lower() == 'exit':
            break
        else:
            
            processed_frame, target_position = process_frame(camera_matrix, dist_coeffs, object_d) 
    
            if target_position:
                if object_d == 0.043:  # Ball
                    print(f"Ball detected at: {target_position}, Robot Z-coordinates: {target_position[3][2]}")
                    move_robot_to_target(target_position[2], target_position[0])
                    print("Moving robot to target...")
                    time.sleep(2)  # Turn right 90 degrees to align with the hole
                    service.send(service.topicCmd + "T0/servo","1 -200 200")
                    time.sleep(1)
                    service.send(service.topicCmd + "T0/servo","1 8000 0") #Relax arm
                    #break

                elif object_d == 0.052:  # Hole
                    print(f"Hole detected at: {target_position}, Robot Z-coordinates: {target_position[3][2]}")
                    move_robot_to_target(target_position[2], target_position[0])
                    print("Moving robot to target...")
                    #break
                
            else:
                print("No ball or hole detected.")
                continue
            
        



if __name__ == "__main__":
    setproctitle("mqtt-client")
    print("% Starting")
    
    # Setup service (assuming you have already set up the service in your script)
    #service.setup('localhost')  # localhost, or whatever server address you need
    service.setup('10.197.216.254')
    if service.connected:
        # Run the image capture loop in a separate thread
        image_capture_thread = threading.Thread(target=test_loop(object_d=0.043), daemon=True)
        image_capture_thread.start()

        print("System initialized. Running normally...")
        #turn(-1.57) # Turn left 90 degrees to start
        # Run the main loop which will listen for the exit command only
        test_loop(object_d=0.043)  # Change to 0.052 for hole detection
        time.sleep(0.5)
        
        #image_capture_thread = threading.Thread(target=test_loop(0.052), daemon=True)
        #image_capture_thread.start()
        #turn(1.57)  # Turn right 90 degrees to find hole
        #test_loop(object_d=0.052) # Looking for hole
        

        
        
        
    service.terminate()
    print("% Main Terminated")