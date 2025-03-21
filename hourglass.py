from uservice import service
from sir import ir
from simu import imu
from sedge import edge
from spose import pose
import numpy as np


distance_to_regbot = 1.5

def hourglass():
    print("Start hourglass")
    done = False
    pose.tripBreset()
    state = 0
    while not (service.stop):
        if state == 0:
            print(ir.ir[1])
            if ir.ir[1] < distance_to_regbot:
                service.send(service.topicCmd + "ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)
                state = 1
                pose.tripBreset()

        elif state == 1:
            if edge.crossingLineCnt > 0 or pose.tripBtimePassed() > 5:
                service.send(service.topicCmd + "ti/rc","0.0 0.5") # (forward m/s, turn-rate rad/sec)
                state = 2
                pose.tripBreset()
        elif state == 2:
            if pose.tripBh > np.pi/4:
                service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
                state = 3
        else:
            print("Done with hourglass")
            break

