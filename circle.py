from uservice import service
from sir import ir
from simu import imu
from sedge import edge
from spose import pose
import numpy as np
from datetime import *

gyro_value = 25
acc_value = 0.95
time_value = 100000 #.001 seconds
amount_of_checks = 1000


def circle():
    print("starting circle")
    pose.tripBreset()
    state = 0
    detect_times = 0
    time_at_detect = datetime.now()
    normal_times=0
    while not service.stop:
        if state == 0:
            print("starting driving")
            service.send(service.topicCmd + "ti/rc","0.3 0.0")
            state = 10
        
        elif state == 10:
            print(detect_times)
            #If using gyro, use gyro_value, if using acc, use acc_value. If mixed, I have forgot to change one of them
            if abs(imu.gyro[2]) > gyro_value:
                time_at_detect = datetime.now()
                state = 20
                detect_times += 1

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

