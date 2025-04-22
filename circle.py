from uservice import service
from sir import ir
from simu import imu
from sedge import edge
from spose import pose
import numpy as np
from datetime import *
from detect_object import process_frame, camera_matrix,dist_coeffs, turn

gyro_value = 25
acc_value = 0.95
time_value = 200000 #.001 seconds
amount_of_checks = 1000
turn_speed = 0.5
turn_leeway = 50

image_center = 820/2

def rotate_towards():
    print("rotating")
    x=0
    state = 40
    while not service.stop:
        if state == 40:
            _,_,coords,_ = process_frame(camera_matrix,dist_coeffs,0.1)
            if coords is not None:
                x = coords[0]
            print("Circle", x)
            if x < image_center - turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 0.2")
                state = 51
                #state = 999
            elif x > image_center + turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 -0.2")
                state = 52
                #state = 999
            else:
                state = 60
                #state = 999
        elif state == 51:
            _,_,coords,_ = process_frame(camera_matrix,dist_coeffs,0.1)
            if coords is not None:
                x = coords[0]
            print("Circle 51", x)
            if x > image_center - turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 0.0")
                state = 40
        elif state == 52:
            _,_,coords,_ = process_frame(camera_matrix,dist_coeffs,0.1)
            if coords is not None:
                x = coords[0]
                print("Circle 52", x)
                if x < image_center + turn_leeway:
                    service.send(service.topicCmd + "ti/rc","0.0 0.0")
                    state = 40
        else:
            break


def circle():
    print("starting circle")
    pose.tripBreset()
    state = 0
    while not service.stop:
        if state == 0:
            rotate_towards()
            state = 10

        elif state == 10:
            print(imu.gyro[0])
            service.send(service.topicCmd + "ti/rc","0.3 0.0")
            pose.tripBreset()
            state = 20
        elif state == 20:
            print(imu.gyro[0])
            if imu.gyro[0] < 10 and pose.tripBtimePassed() >= 3:
                service.send(service.topicCmd + "ti/rc","-0.3 0.0")
                state = 30
                pose.tripBreset()
        elif state == 30:
            print(pose.tripBh)
            if pose.tripBtimePassed() > 0.5:
                service.send(service.topicCmd + "ti/rc","0.0 0.0")
                turn(angle=1.4)
                pose.tripBreset()
                state = 40
        elif state == 40:
            print(pose.tripBh)
            service.send(service.topicCmd + "ti/rc","0.2 0.6")
            state = 50
        elif state == 50:
            print(pose.tripBh)
            if pose.tripBh >= np.pi*1.7:
                service.send(service.topicCmd + "ti/rc","0.0 0.0")
                state = 60

        
        else:
            break


def circle_maybe():
    print("starting circle")
    pose.tripBreset()
    state = 0
    detect_times = 0
    time_at_detect = datetime.now()
    normal_times=0
    x = 0
    while not service.stop:
        if state == 0:
            print("starting driving")
            service.send(service.topicCmd + "ti/rc","0.4 0.0")
            state = 10
        
        elif state == 10:
            print(imu.gyro[0])
            if imu.acc[0] < -1: # If the robot stops driving forward, it means that it has run into the gates
                # If it has run into the gates, back up a bit, then continue.

                service.send(service.topicCmd + "ti/rc","-0.2 0.0")
                pose.tripBreset()
                state = 15
            #If using gyro, use gyro_value, if using acc, use acc_value. If mixed, I have forgot to change one of them
            elif abs(imu.gyro[2]) > gyro_value:
                time_at_detect = datetime.now()
                state = 20
                detect_times += 1
        elif state == 15:
            print(pose.tripBh)
            if pose.tripBtimePassed() >= 1:
                service.send(service.topicCmd + "ti/rc","0.0 0.0")
                pose.tripBreset()
                state = 40


        elif state == 20:
            print((datetime.now()-time_at_detect).microseconds)
            if abs(imu.gyro[2]) > gyro_value:
                print("Hello")
                time_at_detect = datetime.now()
            elif (datetime.now()-time_at_detect).microseconds > time_value:
                    if detect_times < 2:
                        state = 10
                    else:
                        service.send(service.topicCmd + "ti/rc","0.0 0.0")
                        pose.tripBreset()
                        normal_times = 0
                        state = 30
        elif state == 30:
            #Double check if it actually is on the platform
            print("normal times",normal_times)
            if imu.acc[2]>acc_value:
                normal_times += 1
            else:
                normal_times = 0
            
            if normal_times >= amount_of_checks:
                state = 40
            elif pose.tripBtimePassed() > 1:
                detect_times = 0
                time_at_detect = datetime.now()
                normal_times=0
                #Try again if a second has passed, and it seems like there is still tilt
                state = 0
        
        elif state == 40:
            _,_,coords,_ = process_frame(camera_matrix,dist_coeffs,0.1)
            if coords is not None:
                x = coords[0]
            print("Circle", x)
            if x < image_center - turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 0.2")
                state = 51
                #state = 999
            elif x > image_center + turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 -0.2")
                state = 52
                #state = 999
            else:
                state = 60
                #state = 999
        
        elif state == 51:
            _,_,coords,_ = process_frame(camera_matrix,dist_coeffs,0.1)
            if coords is not None:
                x = coords[0]
            print("Circle 51", x)
            if x > image_center - turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 0.0")
                state = 40
        elif state == 52:
            _,_,coords,_ = process_frame(camera_matrix,dist_coeffs,0.1)
            if coords is not None:
                x = coords[0]
            print("Circle 52", x)
            if x < image_center + turn_leeway:
                service.send(service.topicCmd + "ti/rc","0.0 0.0")
                state = 40

        elif state==60:
            turn(angle=1.57)
            service.send(service.topicCmd + "ti/rc","0.2 0.6 ")
            pose.tripBreset()
            state = 70
        
        elif state==70:
            print(pose.tripBh)
            if pose.tripBh >= np.pi*1.7:
                pose.tripBreset()
                state = 80
                service.send(service.topicCmd + "ti/rc","0.0 0.0")

        
        else:
            service.send(service.topicCmd + "ti/rc","0.0 0.0")
            break

def test_acc():
    pose.tripBreset()
    state = 0
    while not service.stop:
        if state == 0:
            print(imu.acc[2])


def circle_old():
    print("starting circle")
    pose.tripBreset()
    state = 0
    peaks = 0
    times_out_of_peaks = 0
    while not service.stop:
        if state==0:
            print("driving")
            service.send(service.topicCmd + "ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)

            if abs(imu.gyro[2]) > 30:
                peaks += 1
                state = 10
            
            if peaks == 2:
                state = 50
        
        elif state == 10:
            print("Detected peak")
            if abs(imu.gyro[2]) < 20:
                times_out_of_peaks += 1
            else:
                times_out_of_peaks = 0
            if times_out_of_peaks == 50:
                state = 0


        elif state == 20:
            service.send(service.topicCmd + "ti/rc","0.2 0.0")
            state = 50
        elif state == 50:
            print("Stopping")
            if pose.tripBtimePassed() > 0:
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                state= 999
        else:
            print("done with circle.")
            break

