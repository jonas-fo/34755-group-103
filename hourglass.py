from uservice import service
from sir import ir
from simu import imu
from sedge import edge
from spose import pose
import numpy as np


distance_to_regbot = 0.5
distance_to_gate = 0.2
minimum_dist_to_regbot = 0.1


desired_stop_heading = -0.4

hourglass_state = 0

def hourglass():
    print("Start hourglass")
    done = False
    pose.tripBreset()
    hourglass_state = -1
    prev_state = 0
    while not (service.stop):
        if hourglass_state == -1:
            print(ir.ir[1])
            if ir.ir[1] < distance_to_regbot:
                hourglass_state = 0
        elif hourglass_state == 0:
            print(ir.ir[1])
            if ir.ir[1] > distance_to_regbot + 0.1: # Add a little, in case of noise
                service.send(service.topicCmd + "ti/rc","0.3 0.0") # (forward m/s, turn-rate rad/sec)
                hourglass_state = 1
                pose.tripBreset()

        elif hourglass_state == 1:
            print(edge.lineValidCnt)
            if edge.lineValidCnt > 0 or pose.tripBtimePassed() > 7:
                service.send(service.topicCmd + "ti/rc","0.0 0.5") # (forward m/s, turn-rate rad/sec)
                hourglass_state = 2
                pose.tripBreset()
            elif ir.ir[1] < distance_to_regbot: # If the regbot was detected to be gone due to noise, stop and wait again
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                hourglass_state = 0
        elif hourglass_state == 2:
            print(pose.tripBh)
            if pose.tripBh > np.pi/4:
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                hourglass_state = 3
                edge.lineControl(0.15, 1.3) # m/s and position on line -2.0..2.0
        elif hourglass_state == 3:
            print(pose.tripBh)
            if pose.tripBh <= desired_stop_heading:# or ir.ir[0] < distance_to_gate:
                hourglass_state=4
            elif ir.ir[1] < minimum_dist_to_regbot:
                edge.lineControl(0.0,0.0)
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                prev_state = hourglass_state
                hourglass_state = 10
        elif hourglass_state == 4:
            print(pose.tripBh)
            if pose.tripBh <= desired_stop_heading:# or ir.ir[0] > distance_to_gate:
                hourglass_state = 5
            elif ir.ir[1] < minimum_dist_to_regbot:
                edge.lineControl(0.0,0.0)
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                prev_state = hourglass_state
                hourglass_state = 10
        elif hourglass_state == 5:
            print(pose.tripBh)
            if pose.tripBh <= desired_stop_heading:# or ir.ir[0] < distance_to_gate:
                edge.lineControl(0.0,0.0)
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                hourglass_state = 6
            elif ir.ir[1] < minimum_dist_to_regbot:
                edge.lineControl(0.0,0.0)
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                prev_state = hourglass_state
                hourglass_state = 10

        elif hourglass_state == 10:
            #The robot has gotten too close to the regbot.
            if ir.ir[1] > minimum_dist_to_regbot:
                edge.lineControl(0.15, 1.0) # m/s and position on line -2.0..2.0
                hourglass_state=prev_state
        elif hourglass_state == 999: # Test state
            print(pose.tripBh)
            pass
        else:
            print("Done with hourglass with heading: ", pose.tripBh)
            break
