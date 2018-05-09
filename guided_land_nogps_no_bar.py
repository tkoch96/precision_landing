from time import time,sleep
import numpy as np
from math import pi
from dronekit import VehicleMode, connect
from MyCopter import MyCopter
from Image_Processor import Image_Processor

print("Start timer.")
t_start = time()

def wrap_360(radian_measurement):
    #convert a radian measurement to a degree measurement in the interval [0,360]
    if(radian_measurement < 0 ):
        radian_measurement += 2 * pi
    return radian_measurement * 180 / pi

def run_test_basic_precland(cop,im_proc):
    #### main loop
    cop.vehicle.mode = VehicleMode("LAND")
    while(True):
        #get an image from the video camera and command the copter
        im_proc.loop()
        if cop.vehicle.armed == False and not cop.indoor_mode: #if we are landed, exit the program
           break

def run_test_basic_rotate(cop,im_proc):
    cop.rotate_drone_heading()
    sleep(3)
    print("Switching to land mode.")
    cop.vehicle.mode = VehicleMode("LAND")
    while(True):
        #get an image from the video camera and command the copter
        im_proc.loop()
        if cop.vehicle.armed == False and not cop.indoor_mode: #if we are landed, exit the program
            break

def run_rotate_precland(cop,im_proc):
    #### main loop
    cop.vehicle.mode = VehicleMode("LAND")
    t_last_rotate = time()
    t_start = time()
    while(True):
        #get an image from the video camera and command the copter
        im_proc.loop()
        if(time() - t_last_rotate > 4 and time() - t_start > 3):
            cop.rotate_drone_heading()
            t_last_rotate = time()
        if cop.vehicle.armed == False and not cop.indoor_mode: #if we are landed, exit the program
            break

def main():
    debug_mode = True #should we take the time to record video? This eats some frames
    indoor_mode = False #turn off thrust and other things

    vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)

    #flight parameters
    go_up = 1.1
    target_altitude = vehicle.location.global_relative_frame.alt + go_up
    if target_altitude > go_up:
       target_altitude = go_up

    #### perform pre-arm checks, switch modes
    vehicle.flush()
    cop = MyCopter(vehicle,indoor_mode)
    im_proc = Image_Processor(cop, t_start, debug_mode)
    #if not indoor_mode:
    im_proc.launch_copter(target_altitude)
    
    print("Switching to land mode")
    ### reached target altitude - now lan
    #run_test_basic_rotate(cop,im_proc)
    #run_test_basic_precland(cop,im_proc)
    #run_test_paused_rotate(cop, im_proc)
    run_rotate_precland(cop,im_proc)
    #run_rotate_precland_adv(cop,im_proc)

    #once landed/done, disarm and release resources
    vehicle.armed = False
    im_proc.teardown()
    

if __name__ == "__main__":
    main()
