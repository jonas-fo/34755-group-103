

def RightSide_follow(speed,time):
  while (pose.tripBtimePassed() < time or edge.lineValidCnt == 0):
    edge.lineControl(speed, -0.5) # keeps the line in the middle at  'speed'cm/s
    #-1 might have to be changed to +1 depending on what wide of the line sensor is + and -
  return

def RampDown():
  while(imu.gyro[0] < 2):
    service.send(service.topicCmd + "ti/rc","0.2 0") # go straight
  service.send(service.topicCmd + "ti/rc","0 0")