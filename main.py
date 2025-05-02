#from group_103 import main

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

#import sys
#import threading
import time as t
import math
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service
from simu import imu
#import Seesaw
from hourglass import hourglass
import detect_object
import circle
import aruco_navigator




############################################################

def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      edge.paint(img)
      
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass

############################################################

stateTime = datetime.now()
def stateTimePassed():
  return (datetime.now() - stateTime).total_seconds()

############################################################

def driveOneMeter():
  state = 0
  pose.tripBreset()
  print("# Driving 1m -------------------------")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not (service.stop or gpio.stop()):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        state = 99
    else:
      print(f"# drive 1m drove {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
    t.sleep(0.05)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("# Driving 1m ------------------------- end")

def driveToLine():
  state = 0
  pose.tripBreset()
  dist_to_line = 0;
  print("% Driving to line ---------------------- right ir start ---")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not (service.stop):
    if state == 0: # forward towards line
      if ir.ir[0] < 0.2:
        service.send("robobot/cmd/ti/rc","0.2 0.0") # (forward m/s, turn-rate rad/sec)
        service.send("robobot/cmd/T0/lognow","3") # (start Teensy log)
        state = 1
    elif state == 1:
      if pose.tripB > 1.0 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      if edge.lineValidCnt > 4:
        # start follow line
        edge.lineControl(0.2, 0)
        dist_to_line = pose.tripB
        pose.tripBreset()
        print(" to state 10")
        state = 10
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001:
        print(" to state 99")
        state = 99
    elif state == 10:
      if edge.lineValidCnt < 2:
        edge.lineControl(0, 0)
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        print(" to state 2")
        state = 2
    else:
      print(f"# drive to line {dist_to_line:.3f}m, then along line {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    # print(f"# drive {state}, now {pose.tripB:.3f}m in {pose.tripBtimePassed():.3f} seconds, line valid cnt = {edge.lineValidCnt}")
    t.sleep(0.01)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("% Driving to line ------------------------- end")


def driveTurnPi():
  state = 0
  pose.tripBreset()
  print("# Driving a Pi turn -------------------------")
  service.send(service.topicCmd + "T0/leds","16 0 100 0") # green
  while not (service.stop or gpio.stop()):
    if state == 0: # wait for start signal
      service.send("robobot/cmd/ti/rc","0.2 0.5") # (forward m/s, turn-rate rad/sec)
      state = 1
    elif state == 1:
      if pose.tripBh > 3.14 or pose.tripBtimePassed() > 15:
        service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        state = 2
      pass
    elif state == 2:
      if abs(pose.velocity()) < 0.001 and abs(pose.turnrate()) < 0.001:
        state = 99
    else:
      print(f"# drive turned {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
      service.send("robobot/cmd/ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
      break;
    print(f"# turn {state}, now {pose.tripBh:.3f} rad in {pose.tripBtimePassed():.3f} seconds")
    t.sleep(0.05)
  pass
  service.send(service.topicCmd + "T0/leds","16 0 0 0") # end
  print("# Driving a Pi turn ------------------------- end")

def loop():
  from ulog import flog
  state = 0 if service.args.action == None else service.args.action
  print("Initial state = ", state)
  images = 0
  ledon = True
  tripTime = datetime.now()
  oldstate = -1
  service.send(service.topicCmd + "T0/leds","16 30 30 0") # LED 16: yellow - waiting
  if service.args.meter:
    state = 101 # run 1m
  elif service.args.pi:
    state = 102 # run 1m
  elif service.args.distance:
    state=500
  elif not service.args.now:
    print("% Ready, press start button")
    while(not gpio.get_value(19)):
      pass
  # main state machine
  edge.lineControl(0, 0) # make sure line control is off
  while not (service.stop):
    if state == 0: # wait for start signal
      start = True #= gpio.start() or service.args.now
      if start:
        print("% Starting")
        service.send(service.topicCmd + "T0/leds","16 0 0 30") # blue: running
        service.send(service.topicCmd + "ti/rc","0.0 0.0") # (forward m/s, turn-rate rad/sec)
        # follow line (at 0.25cm/s)
        edge.lineControl(0.15, -1.0) # m/s and position on line -2.0..2.0
        #service.send(service.topicCmd + "T0/servo","1 -200 200")
        #robobot/drive/T0/servo 1 400 200
        state = 12 # until no more line
        pose.tripBreset() # use trip counter/timer B
    elif state == 12: # following line
      #imu.print()
      #imu.decode("")
      print("Line valid count",edge.lineValidCnt)
      if pose.tripBtimePassed() > 20:#edge.lineValidCnt == 0 or pose.tripBtimePassed() > 25: #skift time tilbage til 15
        edge.lineControl(0.17, 0.5) # m/s and position on line -2.0..2.0
        pose.tripBreset() # use trip counter/timer B
        state = 15
    elif state==15:
      #edge.lineControl(0.20, 0.5)
      print("crossing cnt: ",edge.crossingLineCnt)
      print("Line cnt: ",edge.lineValidCnt)
      if edge.crossingLineCnt > 0: #or pose.tripBtimePassed>2:#needs tuning
        print("Crossed line")
        edge.lineControl(0, 0)
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        state = 16
        pose.tripBreset()
      #elif edge.lineValidCnt == 0:
       # state = 14
    elif state==16:
      if pose.tripBtimePassed() > 1: #ud på rampen
         detect_object.turn(angle=-(math.pi/4))
         #service.send(service.topicCmd + "ti/rc","0.1 0.0")
         pose.tripBreset()
         state = 14
    elif state == 14:
      if pose.tripBtimePassed() > 1.2: #needs tuning
         service.send(service.topicCmd + "ti/rc","0.0 0.0")
         service.send(service.topicCmd + "T0/servo","1 -200 0")
         pose.tripBreset()
         state = 17
      
    elif state == 17:
      #if pose.tripBtimePassed()>3:
        edge.lineControl(0,0)
        t.sleep(0.5)
        service.send(service.topicCmd + "T0/servo", "1 -10000 0") #this is perfect for straight arm
        #service.send(service.topicCmd + "T0/servo", "1 -1 0")
        t.sleep(1)
        print("Arm Down!")
      
        #service.send(service.topicCmd + "T0/servo", "1 10000 200") #perfect for resting arm
        #service.send(service.topicCmd + "T0/servo","1 -10000 200")
        edge.lineControl(0.18, 0) #ned af rampen
        state = 18
        pose.tripBreset()
    elif state == 18:
       #find krydset
      if edge.crossingLineCnt > 0 or pose.tripBtimePassed() > 15: #needs tunning
        service.send(service.topicCmd + "T0/servo","1 -900 0")
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc","-0.01 -0.5")
        pose.tripBreset()
        state = 190
    elif state == 190:
      if pose.tripBh > np.pi/8 or pose.tripBtimePassed() > 2.2: 
        service.send(service.topicCmd + "ti/rc","0.2 0.0")
        service.send(service.topicCmd + "T0/servo","1 -10000 0")
        pose.tripBreset()
        state = 19
    elif state == 19: #turning right
      if pose.tripBtimePassed() > 1: 
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")
        edge.lineControl(0.18,0) #finden den store rampe
        pose.tripBreset()
        state = 21
    elif state == 21:
      if edge.crossingLineCnt > 0 or pose.tripBtimePassed() > 3: 
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc","0.25 -0.5")  #drej mod rampen
        pose.tripBreset()
        state = 22
    elif state == 22:
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 2: 
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")
        edge.lineControl(0.20,0) #op af rampen
        pose.tripBreset()
        state = 23
    elif state == 23: 
      if pose.tripBtimePassed() > 11: #do sweep to stairs instead
        edge.lineControl(0,0)
        service.send(service.topicCmd + "T0/servo","1 -200 200")
        print("Arm down")
        service.send(service.topicCmd + "ti/rc","0.15 -0.5")#needs tuning
        pose.tripBreset()
        state = 28
        



    elif state == 28:
      if pose.tripBtimePassed() > 3.4: #needs tuning
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        service.send(service.topicCmd + "T0/servo","1 -10000 200")
        edge.lineControl(0.2,0)
        pose.tripBreset()
        state = 29
    elif state == 29:
      print("Crossing lines: ",edge.crossingLineCnt)
      if pose.tripBtimePassed() > 20 or edge.crossingLineCnt > 0:
          edge.lineControl(0,0)
          service.send(service.topicCmd + "ti/rc","-0.01 0.5")
          pose.tripBreset()
          state=30
    elif state == 30:
      if pose.tripBtimePassed() > 2:
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        #edge.lineControl(0.17,0)
        dis=1.2
        s=0.17
        ti=dis/s
        edge.lineControl(s,0)
        t.sleep(ti)
        print("TRAVELLLLLLLLLLLLLLLED", dis)
        edge.lineControl(0.12,0)
        pose.tripBreset()
        state = 31
    elif state == 31:
      #print("lines: ",edge.crossingLineCnt)
      #if pose.tripBtimePassed() > 5: #needs tuning
      #  edge.crossingLineCnt
      t.sleep(0.1)
      #print("WHATTTTTTTTTTTTT IS VA>LEEE: ",edge.crossingLineCnt)

      if edge.crossingLineCnt > 0:
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc","-0.01 -0.5")
        pose.tripBreset()
        state = 32
      
    
      
      


            
          
          
          
    elif state == 32:
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 2: #needs tuning
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        service.send(service.topicCmd + "T0/servo","1 -300 0")
        # jónas's 8 tals kode
        pose.tripBreset()
        state = 33
    elif state == 33:
      if pose.tripBtimePassed() > 2: #needs tuning
        service.send(service.topicCmd + "ti/rc","0.1 0.0")
        pose.tripBreset()
        #service.send(service.topicCmd + "T0/servo","1 -10000 0")
        state = 34
    elif state == 34:
      if pose.tripBtimePassed() > 2: #needs tuning
        service.send(service.topicCmd + "ti/rc","-0.1 0.0")
        service.send(service.topicCmd + "T0/servo","1 -900 0")
        pose.tripBreset()
        state = 35
    elif state == 35:
      print("crossing lines: ",edge.crossingLineCnt)
      if edge.crossingLineCnt > 0 or pose.tripBtimePassed() > 2: #needs tuning
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        service.send(service.topicCmd + "ti/rc","-0.01 0.5") # needs tuning
        pose.tripBreset()
        state = 36
    elif state == 36:
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 2: #needs tuning
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        service.send(service.topicCmd + "T0/servo","1 -10000 0")
        edge.lineControl(0.15,0)
        pose.tripBreset()
        state = 37
    elif state == 37:
      print("crossing lines: ",edge.crossingLineCnt)
      if edge.crossingLineCnt > 0: #or pose.tripBtimePassed() > 5:
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc","0.01 -0.5")
        pose.tripBreset()
        state = 38
    elif state == 38:
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 2:
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        edge.lineControl(0.1,0)
        pose.tripBreset()
        state = 39
    elif state == 39:
      if pose.tripBtimePassed() > 2:
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc","0.0 0.0")
        pose.tripBreset()
        state = 199

    elif state == 14: # turning left
      if pose.tripBh > np.pi/2 or pose.tripBtimePassed() > 10:
        state = 20 # finished   =17 go look for line
        service.send(service.topicCmd + "ti/rc","0.0 0.0") # stop for images
        processed_frame, ball_position,_,_ = detect_object.process_frame(detect_object.camera_matrix, detect_object.dist_coeffs,0)
        print("Ball postion: ",ball_position)
      print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")
    elif state == 20: # image analysis
      imageAnalysis(images == 2)
      images += 1
      service.send(service.topicCmd + "T0/servo","1 -10000 200")
      # blink LED
      if ledon:
        service.send(service.topicCmd + "T0/leds","16 0 64 0")
        gpio.set_value(20, 1)
      else:
        service.send(service.topicCmd + "T0/leds","16 0 30 30")
        gpio.set_value(20, 0)
      ledon = not ledon
      # finished?
      if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
        images = 0
        state = 99
      pass
    elif state == 101:
      driveOneMeter();
      state = 100;
    elif state == 102:
      driveTurnPi();
      state = 100;
    elif state == 103:
      driveToLine()
      state = 100
    elif state == 123:
      #Read ir values
      #service.send(service.topicCmd + "T0/sub", "ir 1000")
      hourglass()
      state = 150
    elif state == 124:
      #ir.print()
      #imu.print()
      state = 999
    elif state == 150:
      circle.circle()
      edge.lineControl(0.3,-0.5)
      pose.tripBreset()
      state = 151
    elif state == 151:
      if pose.tripBtimePassed() > 3:
        edge.lineControl(0.0,0.0)
        state = 204
    elif state == 500:
      ir.print()
      # Calibrate distance sensor
      pass
    
    elif state == 199: ################# AXE GATE
      
      detect_object.axe_sequence()
      state=999
      #state=200
      
    elif state ==200: 
      
      service.send(service.topicCmd + "ti/rc", "0.0 0.0")
      service.send(service.topicCmd + "T0/servo", "1 -1000 200")
      t.sleep(0.5)

      
      object_d=0.046 #blue ball
      
      aruco_navigator.object_finder(object_d)
      


      service.send(service.topicCmd + "T0/servo", "1 2 200") #this is perfect for straight arm
      service.send(service.topicCmd + "T0/servo", "1 100 200")
      t.sleep(1)
      print("Arm Down!")
      detect_object.turn(angle=-(math.pi/3)) #turn left to begin seek 
      #service.send(service.topicCmd + "T0/servo", "1 10000 200") #perfect for resting arm
      #t.sleep(0.5)
      #state=202
      state=999
   
      
    
    elif state==202: ## Aruco FINDERRRRR
      
      #service.send(service.topicCmd + "T0/servo", "1 99 200") #this is perfect for straight arm
      #service.send(service.topicCmd + "T0/servo", "1 100 200")
      #t.sleep(1)
      #print("Arm Down!")
      object_d="aruco"
      
      id_C=aruco_navigator.object_finder(object_d) # first try will look for 10,11,12,13  
      if id_C ==10 or id_C == 12 or id_C == 13 or id_C == 18: 
        aruco_navigator.object_finder(object_d) # second try will look for 14,15
      
      else: #meaning we are at 14 or 15 or NOne
        pass 
  
      service.send(service.topicCmd + "T0/servo", "1 -1000 200") # perfect for resting arm UP
      t.sleep(1)
      #state=203
      state=999
      
    elif state == 203: ## Recovery after C quad
      
      service.send(service.topicCmd + "ti/rc", "-0.15 0.0")
      t.sleep(0.5)

      while edge.lineValidCnt < 7:
        service.send(service.topicCmd + "ti/rc", "-0.15 0.0")
        t.sleep(0.1)  # Small delay to avoid spamming the command

      # Line is now detected
      service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # Stop
      t.sleep(0.5)
      detect_object.turn(-(math.pi)/3) #turn left 
      t.sleep(0.5)
      edge.lineControl(0.15, 0.0)
      t.sleep(7)
      state = 204 # Move to the next state
    elif state == 204: #turning left
      if edge.crossingLineCnt>0:
        
        edge.lineControl(0,0)
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # double stop
        t.sleep(0.5)
        detect_object.turn(angle=-(1.52)) #turn left
        t.sleep(0.5)
        service.send(service.topicCmd + "ti/rc", "0.2 0.0")  # double stop
        t.sleep(0.5)
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")  # double stop

        state = 123  # Move to the next state
    
    elif state == 205: # THE FINAL COUNTDOWN
      object_d="aruco_END"
      for i in np.linspace(0, 0.8, 3):
        _, result,_, _=detect_object.process_frame(detect_object.camera_matrix, detect_object.dist_coeffs, object_d=object_d)

        if result is not None:
          detect_object.move_robot_to_target(result[2],result[0], followup=False)
          detect_object.move_robot_step(result[2],0, 0.8)
        else:
          print("NOT ALIGNED WIth aruco 25, Make sure previous state aligns with it")
          
      state = 999
          
          
        
      
      
      
      
      state=999
    
    else: # abort
      print(f"% Mission finished/aborted; state={state}")
      break
    # allow openCV to handle imshow (if in use)
    # images are almost useless while turning, but
    # used here to illustrate some image processing (painting)
    if (cam.useCam):
      imageAnalysis(False)
      key = cv.waitKey(100) # ms
      if key > 0: # e.g. Esc (key=27) pressed with focus on image
        break
    #
    # note state change and reset state timer
    if state != oldstate:
      flog.write(state)
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      print(f"% State change from {oldstate} to {state}")
      oldstate = state
      stateTime = datetime.now()
    # do not loop too fast
    t.sleep(0.1)
    pass # end of while loop
  # end of mission, turn LEDs off and stop
  service.send(service.topicCmd + "T0/leds","16 0 0 0") 
  gpio.set_value(20, 0)
  edge.lineControl(0,0) # stop following line
  service.send(service.topicCmd + "ti/rc","0.0 0.0")
  t.sleep(0.05)
  pass

############################################################

if __name__ == "__main__":
    if service.process_running("mqtt-client"):
      print("% mqtt-client is already running - terminating")
      print("%   if it is partially crashed in the background, then try:")
      print("%     pkill mqtt-client")
      print("%   or, if that fails use the most brutal kill")
      print("%     pkill -9 mqtt-client")
    else:
      setproctitle("mqtt-client")
      print("% Starting")
      # where is the MQTT data server:
      service.setup('localhost') # localhost
      
      #service.setup('10.197.216.254') # Cowboy Bebop
      #service.setup('10.197.217.80') # Newton
      #service.setup('bode.local') # Bode
      if service.connected:
        loop()
      service.terminate()
      print("% Main Terminated")

