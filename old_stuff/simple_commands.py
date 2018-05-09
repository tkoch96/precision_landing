import math, time

import cv2
import numpy as np
from dronekit import VehicleMode, connect

from imutils.video import VideoStream
from imutils.video import FPS


#aspect ratio tolerances of the target
aspect_ratio_min = 1.05
aspect_ratio_max = 1.45

#camera properties
#NOTE - EXPERIMENTALLY MEASURED
horizontal_fov = 54.3
vertical_fov = 37.6
horizontal_fov_rad = horizontal_fov * math.pi / 180
vertical_fov_rad = vertical_fov * math.pi / 180
#focal length (m)
foc = .003039
#from calibration
f_x = 2595.36
f_y = 2616.75
m = (f_x + f_y) / ( 2 * foc)
res_x_high = 3280
res_y_high = 2464
#target properties
targ_w = .2794
targ_h = .2159



#mission settings
target_altitude = 0.4

#for voting on past decisions
num_memory = 100
memory = np.zeros([1,num_memory])

#video stream object
#framerate is about 5fps at this resolution
vs = VideoStream(usePiCamera=True, resolution=(1088,720)).start()
time.sleep(2)
frame = vs.read()
horizontal_resolution = frame.shape[1]
vertical_resolution = frame.shape[0]
c_x_image = horizontal_resolution / 2
c_y_image = vertical_resolution / 2



#file writing
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi',fourcc,20,(horizontal_resolution,vertical_resolution),True)
t_start = time.time()

def send_land_message(c_x, c_y, size_x_px, size_y_px, vehicle):
	#calculates the offset from the center of the image and sends a command to the pixhawk
    #print("Size of target in pixels, x: %d  y: %d"%(size_x_px,size_y_px))	
    #get IMU data
    #altitude data is not reliable
    #alt = vehicle.location.global_relative_frame.alt
    #size of the object in the image sensor
    obj_image_sensor = size_x_px / (horizontal_resolution * m / res_x_high)
    #distance to image
    d_cam_image = targ_w * foc / obj_image_sensor

    pitch = vehicle.attitude.pitch
    roll = vehicle.attitude.roll

    #get the errors in x and y
    #following notation of https://pdfs.semanticscholar.org/c4c7/65500423f23e85d80656ac6418aca5190f24.pdf
    alpha_x = pitch
    alpha_y = roll
    
    #full camera fov
    gamma_x = horizontal_fov_rad
    gamma_y = vertical_fov_rad

    #pixel distances to calculate beta
    pd_T_x_S_x = c_x - c_x_image
    pd_C_x_S_x = c_x_image
    beta_x = np.arctan(np.tan(gamma_x/2) * pd_T_x_S_x / pd_C_x_S_x)
    pd_T_y_S_y = c_y_image - c_y
    pd_C_y_S_y = c_y_image
    beta_y = np.arctan(np.tan(gamma_y/2) * pd_T_y_S_y / pd_C_y_S_y)
    #print("Pixel offset from center of image, x: %d , y: %d"%(pd_T_x_S_x, pd_T_y_S_y))
    #from distance
    alt = np.cos(beta_x+alpha_x) * d_cam_image
    
    #This is measured in the picture frame
    e_x = alt * np.tan(alpha_x + beta_x)
    e_y = alt * np.tan(alpha_y + beta_y)

    #Convert this to the drone's frame, based on the camera orientation on the drone
    #currently this is a rotation by pi/2
    tmp = e_y
    e_y = e_x
    e_x = -tmp

    #print("Beta: %.4f offset x: %d, offset y: %d"%( beta_x*180/math.pi, pd_T_x_S_x, pd_T_y_S_y))
    #print("Current roll: %.4f, pitch: %.4f, altitude: %.4f"%(roll*180/math.pi,pitch*180/math.pi,alt))
    print("Sending message to move relatively to x: %.4f, y: %.4f \n Timestamp %s"%(e_x,e_y,time.time() - t_start))
    #help(vehicle.message_factory.landing_target_encode)
    angle_x = (c_x - horizontal_resolution/2) * horizontal_fov_rad/horizontal_resolution
    angle_y = (c_y - vertical_resolution/2) * vertical_fov_rad/vertical_resolution
    #print("Sending message that our landing platform is at Angle_x: %.4f, Angle_y: %.4f"%(angle_x*180/math.pi,angle_y*180/math.pi))
    
    msg = vehicle.message_factory.landing_target_encode(
        0,       # time_boot_ms (not used)
        0,       # target num (not used)
        8,       # body NED frame, we will tell it to move relative to its current frame
        0, #angle_x
        0, #angle_y
        0, #distance
        0, #size_x (not iused)
        0, #size_y (not used)
        0, 0, 0, #x y z
        [0,0,0,0], #q (not used)
        0, #type (not used)
        0) #position_valid (not used)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def detect_and_command(image,vehicle): 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7,7), 0)
    edged = cv2.Canny(blurred, 50, 150)
    _, cnts, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    status = "Target not Acquired, timestamp: %s"%(time.time() - t_start)
    detected = 0
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, .01 * peri, True)
        if len(approx) >= 4 and len(approx) <= 6:
            #bounding box
            (x,y,w,h) = cv2.boundingRect(approx)
            #rotated rectangle
            rect = cv2.minAreaRect(approx)
            w_act,h_act = rect[1]
            
            #make sure width is the longer dimension
            tmp = h_act
            if h_act > w_act:
                h_act = w_act
                w_act = tmp

            #aspect ratio of detected shape
            aspectRatio = w_act /float(h_act)

            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)


            #check to make sure the characteristics of the shape are what we are looking for
            keepDims = w > 25 and h > 25
            keepSolidity = solidity > .9
            keepAspectRatio = aspectRatio >= aspect_ratio_min and aspectRatio <= aspect_ratio_max
            if keepDims and keepSolidity and keepAspectRatio:
                cv2.drawContours(image, [approx], -1, (0,0,255), 4)
                status = "Target(s) Acquired, timestamp: %s"%(time.time() - t_start)
                detected = 1
                #be confident
                #if np.sum(detected) > num_memory * 1.0 / 2:
                #get the center of the image
                M = cv2.moments(approx)
                (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                (startX, endX) = (int(cX - (w * .15)), int(cX + (w * .15)))
                (startY, endY) = (int(cY - (h * .15)), int(cY + (h * .15)))
                #draw target bounding box (red lines)
                cv2.line(image, (startX, cY), (endX, cY), (0,0,255),2)
                cv2.line(image, (cX, startY), (cX, endY), (0,0,255),2)
                #indicate offset from center of image (blue lines)
                cv2.line(image, (c_x_image, c_y_image), (c_x_image, cY), (255,0,0),1)
                cv2.line(image, (c_x_image, cY), (cX, cY), (255,0,0),1)
                #print("Aspect Ratio: %.4f"%aspectRatio)
                send_land_message(cX, cY, w_act, h_act, vehicle)
    #add to memory
    np.roll(memory,1)
    memory[0] = detected
    cv2.putText(image, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
    #disable imshow for field tests
    #cv2.imshow("Frame",image)
    out.write(image)

class MyCopter():
    def __init__(self,vehicle):
        self.vehicle = vehicle
        self.pre_takeoff_mode = "GUIDED_NOGPS"
    def arm_and_takeoff_nogps(self, aTargetAltitude):
        DEFAULT_TAKEOFF_THRUST = 0.1#0.6#0.7
        SMOOTH_TAKEOFF_THRUST = 0.1#0.55#0.6

        self.vehicle.mode = VehicleMode(self.pre_takeoff_mode)
        while not self.vehicle.mode.name == self.pre_takeoff_mode:
            print("waiting for mode change")
            time.sleep(1)
        print("Changed to vehicle mode %s"%self.vehicle.mode.name)
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming... ")
            time.sleep(1)
        print("Is Armed: %s"%self.vehicle.armed)

        i=0
	landed = False
        thrust = DEFAULT_TAKEOFF_THRUST
        while i <= 500:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print("Altitude: ", current_altitude)
            if current_altitude >= aTargetAltitude*.8:
                print("Reached target altitude")
		landed = True
                break
            elif current_altitude >= aTargetAltitude*.6:
                thrust = SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust=thrust)
            time.sleep(0.1)
            i += 1
	if not landed:
		print("Unable to reach target altitude.")

    def drop(self):
	DROP_THRUST = .2
	self.set_attitude(thrust=DROP_THRUST)	

    def set_attitude(self,roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
        msg = self.vehicle.message_factory.set_attitude_target_encode(
                0,
                0,
                0,
                0b00000000,
                self.to_quaternion(roll_angle, pitch_angle),
                0,
                0,
                math.radians(yaw_rate),
                thrust)
        self.vehicle.send_mavlink(msg)

        if duration != 0:
            modf = math.modf(duration)

            time.sleep(modf[0])
            for x in range(0,int(modf[1])):
                time.sleep(1)
                self.vehicle.send_mavlink(msg)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        t0 = math.cos(math.radians(yaw*.5))
        t1 = math.sin(math.radians(yaw*.5))
        t2 = math.cos(math.radians(roll*.5))
        t3 = math.sin(math.radians(roll*.5))
        t4 = math.cos(math.radians(pitch*.5))
        t5 = math.sin(math.radians(pitch*.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w,x,y,z]



def main():
    fps = FPS().start()
    vehicle = connect('/dev/ttyAMA0',wait_ready=True,baud=57600)

    #perform pre-arm checks, switch modes
    vehicle.flush()
    cop = MyCopter(vehicle)
    cop.arm_and_takeoff_nogps(target_altitude)


    #reached target altitude - now land

    #main loop
    j=0
    while(j<=1500):
        #get an image from the video camera
	tf = time.time()
        frame = vs.read()
        #look for the target, if found point the drone to where it needs to go
        #detect_and_command(frame,vehicle)
	send_land_message(0,0,100,100,vehicle)
	time.sleep(.1)
	
        key = cv2.waitKey(1) & 0xFF
         
        if key == ord("q"):
            break
	#print("Operating frequency, %.2f Hz"%(1/(time.time()-tf)))
        fps.update()
        j +=1 
    
    #tempory for testing
    print("Exited loop, switching to land mode and de-arming, index: %d"%j)
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed = False
    fps.stop()
    cv2.destroyAllWindows()
    out.release()
    vs.stop()

if __name__ == "__main__":
    main()
