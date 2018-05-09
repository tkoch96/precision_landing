import math, numpy as np
from time import sleep, time
from dronekit import connect, VehicleMode, mavutil

class MyCopter():
    def __init__(self,vehicle,indoor_mode):
        self.indoor_mode = indoor_mode
        self.vehicle = vehicle
        self.pre_takeoff_mode = "GUIDED_NOGPS"

        #CAMERA PROPERTIES
        #NOTE - EXPERIMENTALLY MEASURED
        self.horizontal_resolution = 0
        self.vertical_resolution = 0
        
        #full camera fov - these variables are to follow the notation of the paper
        self.horizontal_fov = 0
        self.vertical_fov = 0
        self.horizontal_fov_rad = self.horizontal_fov * math.pi / 180
        self.vertical_fov_rad = self.vertical_fov * math.pi / 180

        self.land_platform_heading = 0
        self.last_recorded_height = 0
        self.min_h_update_period = .2 #seconds
        self.t_last_recorded_height = time()
        self.t_start = time()
        self.land_platform_heading = 0 #assume takeoff facing the same direction as you land
        #self.turn_off_useless_messages()

    def arm_nogps(self):
        self.vehicle.mode = VehicleMode(self.pre_takeoff_mode)
        while not self.vehicle.mode.name == self.pre_takeoff_mode:
            print("waiting for mode change")
            sleep(1)
        print("Changed to vehicle mode %s"%self.vehicle.mode.name)
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming... ")
            sleep(1)
        print("Is Armed: %s"%self.vehicle.armed)

    def takeoff_nogps(self, aTargetAltitude):
        if self.indoor_mode:
            DEFAULT_TAKEOFF_THRUST = .05
            SMOOTH_TAKEOFF_THRUST = .05
        else:
            DEFAULT_TAKEOFF_THRUST = .60
            SMOOTH_TAKEOFF_THRUST = 0.53
        thrust = DEFAULT_TAKEOFF_THRUST
        current_altitude = self.last_recorded_height
        if time() - self.t_last_recorded_height > self.min_h_update_period and time() - self.t_start > 11:
            current_altitude = self.vehicle.location.global_relative_frame.alt
        print("Altitude: ", current_altitude)
        if current_altitude >= aTargetAltitude*.95:
            print("Reached target altitude")
            return True
        elif current_altitude >= aTargetAltitude*.8:
            thrust = SMOOTH_TAKEOFF_THRUST
        self.set_attitude(thrust=thrust)
        return False

    def drop(self):
        #just drop the copter
        DROP_THRUST = .2
        self.set_attitude(thrust=DROP_THRUST)   

    def set_last_recorded_height(self, last_recorded_height):
        self.last_recorded_height = last_recorded_height
        self.t_last_recorded_height = time()

    def set_last_recorded_yaw(self, last_recorded_yaw):
        self.last_recorded_yaw = last_recorded_yaw
        self.t_last_recorded_yaw = time()

    def send_land_message_angular(self,c_x, c_y, d_cam_image): #, size_x_px, size_y_px):
        """Sends landing target message to pixhawk."""
        angle_x = (c_x - self.horizontal_resolution/2) * self.horizontal_fov_rad / self.horizontal_resolution
        angle_y = (c_y - self.vertical_resolution/2) * self.vertical_fov_rad / self.vertical_resolution
        print("Sending message  Angle_x: %.2f, Angle_y: %.2f, Distance: %.2f"%(angle_x*180/math.pi,angle_y*180/math.pi,d_cam_image))
        msg = self.vehicle.message_factory.landing_target_encode(
            0,       # time_boot_ms (not used)
            0,       # target num (not used)
            8,       # body NED frame, we will tell it to move relative to its current frame
            angle_x, #angle_x 
            angle_y, #angle_y
            d_cam_image, #distance
            0, #size_x (not iused)
            0, #size_y (not used)
            0, 0, self.last_recorded_height,  #e_x, e_y, -alt, #x y z #NOT IMPLEMENTED IN ARDUCOPTER
            [0,0,0,0], #q (not used)
            0, #type (not used)
            0) #position_valid (not used)
        self.vehicle.send_mavlink(msg)

    def set_land_plat_heading(self):
        """Sets the orientation of the landing platform."""
        self.land_platform_heading = self.vehicle.attitude.yaw
        if self.land_platform_heading < 0:
            self.land_platform_heading += 2 * math.pi
        print("Setting landing platform heading to %.2f degrees"%(self.land_platform_heading * 180/math.pi))
        msg = self.vehicle.message_factory.param_set_encode(
            0, #not used
            0, #not used
            "PLND_LAND_P_OFF", #parameter to change
            self.land_platform_heading, #data
            10 #data type - floating point
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        sleep(.2)

    def set_attitude(self,roll_angle = 0.0, pitch_angle = 0.0, yaw_rate = 0.0, thrust = 0.5, duration = 0):
        msg = self.vehicle.message_factory.set_attitude_target_encode(
                0,
                0,
                0,
                0b00000100,
                self.to_quaternion(roll_angle, pitch_angle),
                0,
                0,
                math.radians(yaw_rate),
                thrust)
        self.vehicle.send_mavlink(msg)

        if duration != 0:
            modf = math.modf(duration)

            sleep(modf[0])
            for x in range(0,int(modf[1])):
                sleep(1)
                self.vehicle.send_mavlink(msg)

    def rotate_drone_heading(self,amount=None):
        """Rotate the drone to be facing the right way."""
        if amount is None:
            amount = self.last_recorded_yaw - self.land_platform_heading
        if amount < 0:
            amount += 180
        amount = np.minimum(amount,7)
        if amount < 1:
            return
        print("Telling the drone to rotate: %.2f degrees (relatively), currently yaw is measuring %.2f degrees."%(amount, self.last_recorded_yaw))
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            amount,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            1, # param 4, relative offset 1, absolute angle 0
            0, 0, 0
        )    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def set_offsets(self):
        #takes the current roll of the pitch of the copter, sets the offset of the pixhawk on the drone
        #to these values
        #assumes that the copter is on the ground
        pitch_offset = self.vehicle.attitude.pitch * 180 / math.pi
        roll_offset = self.vehicle.attitude.roll * 180 / math.pi
        print("Setting roll to %.4f and pitch to %.4f"%(roll_offset,pitch_offset))
        msg = self.vehicle.message_factory.param_set_encode(
            0, #target system (unused)
            0, #target component (unused)
            "MNT_NEUTRAL_Y", #param
            pitch_offset, #data
            10 #data type - floating point
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        sleep(.5)
        msg = self.vehicle.message_factory.param_set_encode(
            0,
            0,
            "MNT_NEUTRAL_X",
            roll_offset,
            10
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

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

    def turn_off_useless_messages(self):
        useless_message_types = [4]
        for message_type in useless_message_types:
            msg = self.vehicle.message_factory.command_long_encode(
                1,  # target_system_id
                0,  # target_component_id
                MAV_CMD_SET_MESSAGE_INTERVAL,  # command id
                0,  # 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
                message_type,  # param1 - The MAVLink message ID
                interval,  # param2 - The interval between two messages, in microseconds. Set to -1 to disable and 0 to request default rate.
                0,  # param3 - Empty
                0,  # param4 - Empty
                0,  # param5 - Empty
                0,  # param6 - Empty
                0,  # param7 - Empty
            )

            self.vehicle.send_mavlink(msg)
            sleep(.3)


