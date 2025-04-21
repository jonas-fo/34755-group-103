import math
import time
from simu import imu  
from detect_object import turn, move_robot_step
from uservice import service

def get_pitch_from_acc(acc):
    # Calculate pitch from accelerometer
    ax, ay, az = acc
    if az == 0 and ay == 0:
        return 0  # avoid division by zero
    pitch_rad = math.atan2(ax, math.sqrt(ay**2 + az**2))
    pitch_deg = math.degrees(pitch_rad)
    return pitch_deg

def flat_check():
    imu.setup()  # Start receiving data 
    print("Checking slope... (Ctrl+C to exit)")
    try:
        while True:
            acc = imu.acc
            pitch = get_pitch_from_acc(acc) 
            print(f"Pitch: {pitch:.2f}°")   
            if abs(pitch) < 5:  # Consider this flat or "not uphill"
                print("✅ Not going uphill - taking action")
                imu.terminate()
                return True
                # --- your action here ---
            else:
                print("⛰️ Going uphill - holding off")  
            time.sleep(0.1) 
    except KeyboardInterrupt:
        print("Stopped.")   
    imu.terminate()
    
import math
import time

def sweep_hole(sweep_angle_deg=20, sweep_speed=0.8  , forward_speed=0.2, increment=0.035, repetitions=3, pause=0.5):
    # Convert angle from degrees to radians
    sweep_angle_rad = math.radians(sweep_angle_deg)
    move_duration = increment / forward_speed

    for i in range(repetitions):
        # Sweep left
        turn(angle=-sweep_angle_rad, turn_speed=sweep_speed)
        time.sleep(pause)

        ## Return to center
        #turn(angle=sweep_angle_rad, turn_speed=sweep_speed)
        #time.sleep(pause)

        # Sweep right
        turn(angle=(2*sweep_angle_rad), turn_speed=sweep_speed)
        time.sleep(pause)

        # Return to center
        turn(angle=-sweep_angle_rad, turn_speed=sweep_speed)
        time.sleep(pause)

        # Move forward
        service.send(service.topicCmd + "ti/rc", f"{forward_speed} 0.0")
        time.sleep(move_duration)

        # Stop
        service.send(service.topicCmd + "ti/rc", "0.0 0.0")


        


    