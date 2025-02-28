

def RightSide_follow(speed,time):
  while (pose.tripBtimePassed() < time or edge.lineValidCnt == 0):
    edge.lineControl(speed, -1.0) # keeps the line in the middle at  'speed'cm/s
    #-1 might have to be changed to +1 depending on what wide of the line sensor is + and -
    pass
  edge.lineControl(0, 0)
  return 12

def RampDown():
  while(imu.gyro[0] < 2):
    service.send(service.topicCmd + "ti/rc","0.2 0") # turn left
  service.send(service.topicCmd + "ti/rc","0 0")