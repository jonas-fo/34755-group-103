import cv2
import numpy as np
import math
import detect_object
from detect_object import process_frame, turn, move_robot_to_target,move_robot_step, camera_matrix, dist_coeffs
from uservice import service
import time as t,time

def move_in_arc(radius=0.55, arc_fraction=0.5, v=0.3, direction="right"):
    """
    Moves the robot in a circular arc.
    
    send: function to send velocity commands (e.g. send("0.3 -0.3"))
    radius: radius of the circle (meters)
    arc_fraction: portion of the full circle to travel (0.5 = half circle)
    v: linear speed (m/s)
    direction: "right" (clockwise) or "left" (counterclockwise)
    """
    omega = v / radius  # angular speed in rad/s
    if direction == "right":
        omega = -omega  # clockwise

    theta = 2 * math.pi * arc_fraction  # radians to travel
    t_move = abs(theta / omega)  # time in seconds

    # Start motion
    service.send(service.topicCmd + "ti/rc", str(v) + str(omega))
    #send(f"{v} {omega}")
    time.sleep(t_move)
    service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # Stop
    return 

def get_yaw_from_rvec(rvec):
    """Convert rotation vector to yaw angle (around Y-axis)."""
    R, _ = cv2.Rodrigues(rvec)
    yaw = math.atan2(R[2, 0], R[2, 2])  # Yaw angle
    return yaw


def object_finder(object_d, shots=6):
    angle = (3*math.pi/4) / (shots)  # 135 degrees with 6 shots = 22.5 degrees
    for shot in range(shots):
        time.sleep(0.5)

        if object_d == "aruco": ############## This is for aruco
            _, result, rvec, ids = process_frame(
                camera_matrix,
                dist_coeffs,
                object_d="aruco"
            )
            
            time.sleep(0.1)
            if ids==14 or ids==15:# or ids==18: 14 might be best test
                if ids==14: #offest more to the right 
                    print("Found Aruco code 14 try right offset") 
                    move_robot_to_target(result[2], result[0]+0.10,followup=False)
                    #turn(angle=0.1) #offest right
                    move_robot_step((result[2]), 0.0, step_scale=0.85)
                    
                if ids==15: #offest more to the left
                    print("Found Aruco code 15 try left offset") 
                    move_robot_to_target(result[2], result[0],followup=False)
                    turn(angle=-0.08) #offest left
                    move_robot_step((result[2]), 0.0, step_scale=0.95)
            
                
                return  ids#result, rvec, ids
            
            if result is not None: 
                print("Found Aruco code")
                navigate_to_aruco_marker(result[2], rvec, ids, result[0])
                return ids#result, rvec, ids  # Return something consistent

            else:
                turn(angle, turn_speed=1)
                print("Aruco code not found. Turning right...")
                time.sleep(0.5)

        else:  ############## This is for balls
            
            time.sleep(0.5)
            frame, result, _,_ = process_frame(
                camera_matrix,
                dist_coeffs,
                object_d=object_d
            )
            time.sleep(0.1)

            if result is not None:
                for i in np.linspace(0.25, 1.0, 4):
                    time.sleep(0.5)
                    frame, result, _,_ = process_frame(
                        camera_matrix,
                        dist_coeffs,
                        object_d=object_d
                    )
                    if result is not None:
                        time.sleep(0.1)
                        X, Y, Z, robot_coords = result
                        move_robot_step(Z, X, step_scale=i)
                        print("Adjusting toward ball...")
                        time.sleep(1)
                return result, None, None
            elif result is None and shot == 0:
                turn(angle=math.pi/3) #turn right first
                print(" Taking that 60° right turn")
            else:
                turn(-angle)
                print("Ball not detected this frame. Turning left...")
                time.sleep(0.5)
                #print("check if break breaks the 8 runs or whole thing")
                #break
        #return result, None, None
    
    return None, None, None  # Now correctly placed outside the loop


def navigate_to_aruco_marker(Z, rvec, marker_id, X):

    yaw_rad = get_yaw_from_rvec(rvec)
    threshold_dis=0.27
    half_width=0.12
    print(f"[INFO] Marker ID: {marker_id}, Z: {Z:.2f} m, Yaw: {yaw_rad:.2f} rad")
    forward_speed = 0.3  # m/s
     
    motion_plan = {
    10: ["standard_start","turn_left","circle arc10"],
    #10: ["standard_start","turn_right", "move_forward55", "turn_left", "move_forward55", "turn_left"],
    13: ["standard_start","turn_left","circle arc13"],
    #13: ["standard_start","turn_right", "move_forward30", "turn_left", "move_forward55", "turn_left"],
    12: ["circle arc12"],
    14: ["move_to_tarket"],
    15: ["move_to_tarket"],
    #18: ["standard_start","turn_left","circle arc10"],
    }

    actions = motion_plan.get(marker_id, [])
    t.sleep(0.1) #just a breather for processing
    for action in actions:
        if action == "turn_right":
            turn(angle=1.2)
            print("[Step] Turned 90° right")
            t.sleep(0.5)
            
        elif action == "turn_left":
            turn(angle=-1.3)
            print("[Step] Turned 90° left")
            t.sleep(0.5)
            
        elif action == "circle arc12":
            
            

            
            move_robot_to_target(Z, X-0.30)
            
            #align with aruco code:
            #turn(yaw_rad-(3.4))
        
            
            
            print("circle arc for 12")
            move_in_arc(radius=0.70, arc_fraction=0.45, v=0.3, direction="right")
            t.sleep(0.1)
            # turn 145 to comensate
            
        elif action == "circle arc13":
            
            print("circle arc for 133333")
            move_in_arc(radius=0.70, arc_fraction=0.45, v=0.3, direction="right")
            t.sleep(0.1)
            
        elif action == "circle arc10":
            print("circle arc for 10")
            move_in_arc(radius=0.70, arc_fraction=0.52, v=0.3, direction="right")
            t.sleep(0.1)
            
                    
        elif action == "move_to_tarket":
            print("YOU ARE HERE!!!!!!!!!!!!!!!!!!!!!!!!!!")
        elif action =="standard_start":
            # Step 1: Sidestep to align with marker
            side_d = Z * math.sin(yaw_rad)
            t.sleep(0.1) #just a breather for processing
            if yaw_rad<0:
                side_angle = 3.14-1.55-abs(yaw_rad)#math.pi - (math.pi / 2) - yaw_rad  # = pi/2 - yaw_rad
            if yaw_rad>0:
                side_angle = abs(3.14-1.55-yaw_rad)
            print(f"[STEP 1] Turn by {math.degrees(side_angle):.2f}°, then move {side_d:.2f} m sideways")

            # Here you would insert your robot's actual movement code, e.g.:

            turn(side_angle)
            t.sleep(0.1) #just a breather for processing
            # Calculate the distance to move forward
            forward_speed = 0.3  # m/s
            move_time=abs(side_d)/forward_speed
            print(f"Moving forward for {move_time:.2f} seconds...")
            service.send(service.topicCmd + "ti/rc", str(forward_speed)+"0.0")
            t.sleep(move_time)
            service.send(service.topicCmd + "ti/rc", "0.0 0.0")
            print("Arrived at target.")


            # Step 2: Turn to face marker head-on
            if yaw_rad > 0:
                turn(angle=-1.56)
                print("[Step 2]Turning 90 left")
            else:
                turn(angle=1.56)
                print("[Step 2]Turning 90 right")
            t.sleep(0.1) #just a breather for processing
            # Step 3: Move toward the marker (subtract 0.45m for final stop distance)
            heading_d = Z * math.cos(yaw_rad)
            stop_d=(0.30+threshold_dis) ## 30 because thats how deepth the marker is in the box
            t.sleep(0.1) #just a breather for processing
            final_drive = abs(heading_d) - stop_d            
            t.sleep(0.1) #just a breather for processing

            # Step 3: Move toward the marker 
            if final_drive > 0: #if the final distance is greater than 0.5m 
                move_time2=final_drive/forward_speed
                print(f"Moving forward for {move_time2:.2f} seconds...")
                service.send(service.topicCmd + "ti/rc", str(forward_speed)+"0.0")
                t.sleep(move_time2)
                service.send(service.topicCmd + "ti/rc", "0.0 0.0")
                print("Arrived at target.")

                print(f"[STEP 3] Drive forward {final_drive:.2f} m to final position")
            else: # if its too close
                print("too close")
                adjust_distance = -final_drive
                move_time2=adjust_distance/forward_speed
                print(f"Moving forward for {move_time2:.2f} seconds...")
                service.send(service.topicCmd + "ti/rc", str(-forward_speed)+"0.0")
                t.sleep(move_time2)
                service.send(service.topicCmd + "ti/rc", "0.0 0.0")
                print("Arrived at target.")
                print(f"[STEP 3] Drive backwards {adjust_distance:.2f} m to final position")
            t.sleep(0.1) #just a breather for processing
        
        # Step 5: find code 15
        #_,result, _, _=process_frame(camera_matrix=detect_object.camera_matrix, dist_coeffs=detect_object.dist_coeffs, object_d="aruco")
        #move_robot_to_target(result[0], result[2])"""
    return 
