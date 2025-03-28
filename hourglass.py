from uservice import service
from sir import ir
from simu import imu
from sedge import edge
from spose import pose
import numpy as np


distance_to_regbot = 0.5
distance_to_gate = 0.2

def hourglass():
    print("Start hourglass")
    done = False
    pose.tripBreset()
    state = -1
    while not (service.stop):
        if state == -1:
            print(ir.ir[1])
            if ir.ir[1] < distance_to_regbot:
                state = 0
        elif state == 0:
            print(ir.ir[1])
            if ir.ir[1] > distance_to_regbot:
                service.send(service.topicCmd + "ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)
                state = 1
                pose.tripBreset()

        elif state == 1:
            print(edge.lineValidCnt)
            if edge.lineValidCnt > 0 or pose.tripBtimePassed() > 7:
                service.send(service.topicCmd + "ti/rc","0.0 0.5") # (forward m/s, turn-rate rad/sec)
                state = 2
                pose.tripBreset()
        elif state == 2:
            print(pose.tripBh)
            if pose.tripBh > np.pi/4:
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                state = 3
                edge.lineControl(0.15, 1.5) # m/s and position on line -2.0..2.0
        elif state == 3:
            print("inside first gate")
            if ir.ir[0] < distance_to_gate:
                state=4
        elif state == 4:
            print("passed first gate")
            if ir.ir[0] > distance_to_gate:
                state = 5
        elif state == 5:
            print("inide second gate")
            if ir.ir[0] < distance_to_gate:
                edge.lineControl(0.0,0.0)
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                state = 6
        elif state == 999: # Test state
            print(pose.tripBh)
            pass
        else:
            print("Done with hourglass")
            break
