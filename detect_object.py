import cv2
import numpy as np
from scam import cam
from setproctitle import setproctitle
from uservice import service
import threading
import os
import math
import time
from sir import ir
from sedge import edge

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

#to save image processed: 
# Create directory if it doesn't exist
save_path = "/home/local/svn/robobot/mqtt_python/group_103/ball_detect_img"
if not os.path.exists(save_path):
    os.makedirs(save_path)


# Load calibration data 
calib_data = np.load("/home/local/svn/robobot/mqtt_python/group_103/calibration_data.npz")
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
    #ball_d = 0.043  # meters

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

def move_robot_to_target(Z,X, stop_distance=0.30, followup=True):
    
    ###########Method 1 for arriving at ball ###################
    forward_speed=0.15 # m/s
    turn_speed=0.5 # rad/s
    #distance=Z-stop_distance
    print("Z: ",Z)
    if Z<0.0188:
        print("TOO CLOSE TO THE BALL")
        distance=Z-stop_distance
    else:
        distance=math.sqrt((Z**2)-(0.0188**2))-stop_distance

    #use the X,Z for the turn rate for 1 sec:
    angle=math.atan2(X,Z)

    turn_time=abs(angle/turn_speed)
    if angle > 0:
        print("Turning right...")
        service.send(service.topicCmd + "ti/rc", "0.0 " + str(-turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")
        print("Turned right.")
        
    elif angle < 0:
        print("Turning lefft...")
        service.send(service.topicCmd + "ti/rc", "0.0 "+str(turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")
        print("Turned lefft.")
        
    else:
        print("No turning needed.")
        
    
    if followup==True:
         #find the distance to the target after turning (centering)
        if distance <= 0:
            print("Already close enough to the target. OR too close CHECK")
            return
        #if distance > 0.9:
        #    
        #    print("THis is way too Far perform SOME LOGIC HERE")
        #    return
        else:
            move_time=distance/forward_speed

            print(f"Moving forward for {move_time:.2f} seconds...")

            service.send(service.topicCmd + "ti/rc", str(forward_speed)+"0.0")
            time.sleep(move_time)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")
            print("Arrived at target.")
            return
    else: 
        print("followup is false, not moving forward") 
        return
    

def process_frame(camera_matrix, dist_coeffs, object_d): 
    save=True
    mask=None
    """Captures, undistorts, and detects a golf ball."""
    with capture_lock:
        ok, frame, imgTime = cam.getImage()
        if not ok:
            print("% Failed to get image.")
            return None, None, None, None  # Return None for both
        
        ####### USE FOR CLEARING IMAGES :Clear existing images in the folder before saving the new one
        #for file in os.listdir(save_path):
        #    file_path = os.path.join(save_path, file)
        #    if os.path.isfile(file_path):
        #        os.remove(file_path)  # Remove the old image

        # Undistort the frame
        undistorted_frame = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix,)
        height, width = undistorted_frame.shape[:2]
        y_split = height // 2
        bottom_half = undistorted_frame[y_split:, :]  # Bottom half of the frame


        if object_d == 0.043: ##### Golf ball
            # Convert to HSV and apply thresholding
            hsv = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([5, 100, 120]) # for the golf ball
            upper_orange = np.array([10, 255, 255]) # for the golf ball
            mask = cv2.inRange(hsv, lower_orange, upper_orange)
            
        
        elif object_d == 0.046: ##### Blue ball
            hsv = cv2.cvtColor(bottom_half, cv2.COLOR_BGR2HSV)
            lower_light_blue = np.array([90, 50, 150])   # Light blue lower bound
            upper_light_blue = np.array([105, 255, 255]) # Light blue upper bound
            
            #lower_light_blue = np.array([99, 40, 82])   # CAlibrated
            #upper_light_blue = np.array([180, 255, 255]) # CAlibrated
            
            mask = cv2.inRange(hsv, lower_light_blue, upper_light_blue)
            
        elif object_d == "aruco": #doesnt need pixel to world conversion
            #mask = None
            gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        
            time.sleep(0.1) #just a breather for processing
            valid_ids = [10,12,13,14,15] # i removed 12 and 18 and 14

            if ids is not None and len(corners) > 0:
                ids = ids.flatten()

                # Check if any valid ID is present
                intersecting_ids = [id for id in ids if id in valid_ids]

                if intersecting_ids:
                    # Pick the first valid one (or choose based on priority)
                    chosen_id = sorted(intersecting_ids)[0] #intersecting_ids[0]  # Or use sorted() to always pick lowest
                    index = np.where(ids == chosen_id)[0][0]
                    print("Chosen ArUco ID:", chosen_id)
                    # Estimate pose for all detected markers
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, 0.1, camera_matrix, dist_coeffs
                    )

                    tvec = tvecs[index][0]
                    rvec = rvecs[index][0]

                    # Draw only the chosen marker
                    cv2.aruco.drawDetectedMarkers(undistorted_frame, [corners[index]], np.array([[ids[index]]], dtype=np.int32))
                    cv2.drawFrameAxes(undistorted_frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                    X_world, Y_world, Z_world = tvec[0], tvec[1], tvec[2]
                    Robot_coor = None

                    return undistorted_frame, (X_world, Y_world, Z_world, Robot_coor), rvec, chosen_id
            else:
                print("No ArUco marker detected.")
                return undistorted_frame, None, None, None
            
        elif object_d == "aruco_END": #doesnt need pixel to world conversion
            #mask = None
            gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
        
            time.sleep(0.1) #just a breather for processing
            valid_ids = [25] # i removed 12 and 18 and 14

            if ids is not None and len(corners) > 0:
                ids = ids.flatten()

                # Check if any valid ID is present
                intersecting_ids = [id for id in ids if id in valid_ids]

                if intersecting_ids:
                    # Pick the first valid one (or choose based on priority)
                    chosen_id = sorted(intersecting_ids)[0] #intersecting_ids[0]  # Or use sorted() to always pick lowest
                    index = np.where(ids == chosen_id)[0][0]
                    print("Chosen ArUco ID:", chosen_id)
                    # Estimate pose for all detected markers
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, 0.1, camera_matrix, dist_coeffs
                    )

                    tvec = tvecs[index][0]
                    rvec = rvecs[index][0]

                    # Draw only the chosen marker
                    cv2.aruco.drawDetectedMarkers(undistorted_frame, [corners[index]], np.array([[ids[index]]], dtype=np.int32))
                    cv2.drawFrameAxes(undistorted_frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                    X_world, Y_world, Z_world = tvec[0], tvec[1], tvec[2]
                    Robot_coor = None

                    return undistorted_frame, (X_world, Y_world, Z_world, Robot_coor), rvec, chosen_id
    
        elif object_d == 0.1: #### Other object
            hsv = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 150, 100])
            upper_red1 = np.array([10, 255, 255])

            lower_red2 = np.array([170, 150, 100])
            upper_red2 = np.array([180, 255, 255])

            # Create masks and combine them
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
        
        

        
        #else:
        #    print(f"[ERROR] Unknown object_d value: {object_d}")
        #    return undistorted_frame, None, None, None
        contours=None   
        if mask is not None:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            print("No contours detected.")
            return undistorted_frame, None, None, None

        
        # Filter contours based on area and shape
        valid_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            print(area)
            if area < 500:  # Reject tiny contours
                continue
            
            ## Optional: shape filter - reject elongated or non-circular
            #perimeter = cv2.arcLength(c, True)
            #if perimeter == 0:
            #    continue
            #circularity = 4 * np.pi * (area / (perimeter * perimeter))
            #if circularity < 0.5:  # 1 is perfect circle, so filter noisy ones
            #    continue
            
            valid_contours.append(c)

        if not valid_contours:
            print("No valid ball-like contours.")
            return undistorted_frame, None, None, None

        # Sort valid contours by how low (y) they are in the frame (prioritize closer ones)
        valid_contours = sorted(valid_contours, key=lambda c: cv2.minEnclosingCircle(c)[0][1], reverse=True)
        chosen_contour = valid_contours[0]
        # Shift the chosen_contour vertically by y_split before drawing
        chosen_contour_shifted = chosen_contour.copy()
        chosen_contour_shifted[:, 0, 1] += y_split  # Shift y-coordinates
        

        (x, y), radius = cv2.minEnclosingCircle(chosen_contour)
        y += y_split  # Adjust Y to match full frame

        
        if cv2.contourArea(c) > 1000:  # Minimum area threshold
            cv2.drawContours(undistorted_frame, [chosen_contour_shifted], -1, (0, 255, 0), 3)

        if radius > 10:
            x, y, radius = int(x), int(y), int(radius)

            # Convert to real-world coordinates
            X_world, Y_world, Z_world, Robot_coor = pixel_to_world(x, y, camera_matrix, radius, object_d)

            # Draw detected ball
            cv2.circle(undistorted_frame, (x, y), radius, (0, 255, 0), 2)
            
            if save:
                # Saving the image with a timestamped filename
                
                img_name = os.path.join(save_path, f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}.jpg")
                cv2.imwrite(img_name, mask)
                #cv2.imshow("Saved Image", undistorted_frame)
                #print(f"% Saved image {img_name}")
            return undistorted_frame, (X_world, Y_world, Z_world, Robot_coor), (x,y), None
        
            
            
        else:
            print(f"[ERROR] Unknown object_d value: {object_d} or no valid contours found.")
            return undistorted_frame, None, None, None

       
         
        



def move_robot_step(z, x, step_scale=0.5):
    # Instead of going full distance, just go a fraction of the way
    z_step = z * step_scale
    x_step = x * step_scale
    move_robot_to_target(z_step, x_step)

       
def turn(angle, turn_speed=1): # 90 degrees is 1.57 radians and -90 degrees is -1.57 radians i think 
    
    turn_time = abs(angle / turn_speed)
    if angle > 0:
        print("Turning right...")
        service.send(service.topicCmd + "ti/rc", "0.0 " + str(-turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")
        print("Turned right.")
        time.sleep(0.1)
    elif angle < 0:
        print("Turning left...")
        service.send(service.topicCmd + "ti/rc", "0.0 " + str(turn_speed))
        time.sleep(turn_time)
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")
        time.sleep(0.1)
        print("Turned left.")
    else:
        print("No turning needed.")
        
        
def axe_sequence():
    
    ## AXE GATE
    gate_distance = 0.5
    
    saw_gate_once = False  # Step 1: wait until we see the axe gate
    while True:
        axe = ir.ir[1]
        #print("IR distance:", axe)
        if not saw_gate_once:
            if axe < gate_distance:  # Detected the axe gate
                print("Axe gate detected!")
                saw_gate_once = True
        else:
            if axe > gate_distance:  # Axe gate has passed
                print("Axe gate confirmed open. Proceeding.")
                break
    
        
    time.sleep(0.05)            
    edge.lineControl(0.8, 0.0) # stop following line
    time.sleep(0.15)
    
    # DECELERATING
    for i in np.arange(0.5, 0.2, -0.1): ## suggestion for slowing down with edge control 
        edge.lineControl(i, 0.0)
        #service.send(service.topicCmd + "ti/rc", str(i)+"0")
        time.sleep(0.5)
        
    #FIND THE END OF THE LINE    
    #edge.lineControl(0, 0)
    while edge.lineValidCnt > 5:
        print("Line valid count: ",edge.high)
        print("Line thress: ",edge.lineValidThreshold)
        edge.lineControl(0.15, 0.0)
        print("Line valid count: ",edge.lineValidCnt)
      
      
    edge.lineControl(0.0, 0.0)
    time.sleep(0.1)
    turn(angle=(math.pi/2)) #turn right
    time.sleep(0.1)
    
    distance=0.60 #change as you see fit
    speed=0.3
    timewait=distance/speed
    
    service.send(service.topicCmd + "ti/rc", str(speed)+ "0.0")
    time.sleep(timewait)        
     
        
def test_loop():
    """ Runs the main loop: captures frames, processes them, and detects the golf ball. """
    
    while True:
        user_input = input("Press Enter to move to object, 1 to capture image or type 'exit' to quit: ")
        if user_input.lower() == 'exit':
            break
        elif user_input.lower() == '1':
            processed_frame, ball_position, _ ,_= process_frame(camera_matrix, dist_coeffs,0.052)
            print("hole at: ", ball_position)
            move_robot_to_target(ball_position[2],ball_position[0])
            print("Moving robot to target...")
        elif user_input.lower() == '2':
            processed_frame, ball_position, _,_ = process_frame(camera_matrix, dist_coeffs,0.1)
        elif user_input.lower() == '0':
            processed_frame, ball_position, _,_ = process_frame(camera_matrix, dist_coeffs,0.043)
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
    #service.setup('10.197.216.254')
    if service.connected:
        ## Run the image capture loop in a separate thread
        #image_capture_thread = threading.Thread(target=test_loop, daemon=True)
        #image_capture_thread.start()
        #print("System initialized. Running normally...")
        # Run the main loop which will listen for the exit command only
        #test_loop()
        
        ############## RUN TO TEST PROCESSFRAME:
        try:
            while True:
                frame, result, _,_ = process_frame(camera_matrix, dist_coeffs, object_d=0.053)

                if frame is not None:
                    cv2.imshow("Live Ball Detection", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break  # Press 'q' to exit
                
        finally:
            cv2.destroyAllWindows()

        
        
        
    service.terminate()
    print("% Main Terminated")