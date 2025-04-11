from uservice import service
from sir import ir
from simu import imu
from sedge import edge
from spose import pose
import numpy as np


def circle():
    print("starting circle")
    pose.tripBreset()
    state = 0
    peaks = 0
    times_out_of_peaks = 0
    while not service.stop:
        if state==0:
            print("driving")
            service.send(service.topicCmd + "ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)

            if imu.acc[2] < -0.5:
                peaks += 1
                state = 50
            
            if peaks == 4:
                state = 50
        
        elif state == 10:
            print("Detected peak")
            if imu.gyro[2] < 20:
                times_out_of_peaks += 1
            else:
                times_out_of_peaks = 0
            if times_out_of_peaks == 15:
                state = 0


        elif state == 50:
            print("Stopping")
            if pose.tripBtimePassed() > 0:
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                state= 999
        else:
            print("done with circle.")
            break

