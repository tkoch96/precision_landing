import cv2, numpy as np
import math
from time import time,sleep
from imutils.video import VideoStream, FPS
import target
import cv2.aruco as aruco

class seen_targ():
	"""Little helper class."""
	def __init__(self,targ,dictionary):
		#generic properties
		self.properties = targ
		self.corners = None
		self.r_mat = None
		self.t_vec = None
		self.position = None
		self.yaw = 0
		self.camera_frame_loc = None
		self.d_cam_image = None
		self.props_are_set = False
		self.yaw = None

		self.board = aruco.GridBoard_create(self.properties["num_x"],self.properties["num_y"],
			self.properties["size_square"],self.properties["space_between"],dictionary,self.properties["marker_start"])
		self.ids = self.board.ids

class Image_Processor():
	def __init__(self, cop, t_start, debug_mode):
		self.cop = cop
		self.t_start = t_start
		self.debug_mode = debug_mode
		self.cop.set_last_recorded_yaw(0)	

		#for the detection of the target
		self.detected = 0
		self.t_last_seen = time()
		self.cX = 0
		self.cY = 0
		self.frame = []

		self.vs = None
		self.foc = .003039 #focal length
		self.set_resolution("med",video_file = "takeoff.avi")
		self.last_recorded_height = 10
		self.last_recorded_height_m = np.array([])

		#aruco targets
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		self.parameters =  aruco.DetectorParameters_create()
        
        #target object setups (actually boards)
		self.targs = []
		for t in target.aruco_boards:
			self.targs.append(seen_targ(t,self.aruco_dict))
		self.color_targs = target.color_targets

		

	def detect_colors(self, targ):
	    #detects a plain piece of paper on the ground, sends a command to the drone to fly to this position

	    hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

	    # construct a mask for the color "white", then perform
	    # a series of dilations and erosions to remove any small
	    # blobs left in the mask
	    mask = cv2.inRange(hsv, targ["color_lower"], targ["color_higher"])
	    mask = cv2.erode(mask, None, iterations=2)
	    mask = cv2.dilate(mask, None, iterations=2)
	    # find contours in the mask and initialize the current
	    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	        cv2.CHAIN_APPROX_SIMPLE)[-2]

	    status = "Target not Acquired"
	    self.detected = 0
	    for c in cnts:
	        peri = cv2.arcLength(c, True)
	        approx = cv2.approxPolyDP(c, .01 * peri, True)
	        if len(approx) >= 4 and len(approx) <= 6:
	            #bounding box
	            (x,y,w,h) = cv2.boundingRect(approx)
	            #rotated rectangle
	            rect = cv2.minAreaRect(approx)
	            self.w_act,self.h_act = rect[1]
	        
	            #make sure width is the longer dimension
	            tmp = self.h_act
	            if self.h_act > self.w_act:
	                self.h_act = self.w_act
	                self.w_act = tmp

	            #aspect ratio of detected shape
	            aspectRatio = self.w_act /float(self.h_act)

	            area = cv2.contourArea(c)
	            hullArea = cv2.contourArea(cv2.convexHull(c))
	            solidity = area / float(hullArea)


	            #check to make sure the characteristics of the shape are what we are looking for
	            keepDims = w > 25 and h > 25
	            keepSolidity = True #solidity > .9
	            keepAspectRatio = aspectRatio >= targ["aspect_ratio_min"] and aspectRatio <= targ["aspect_ratio_max"]
	            #print(keepDims,",",keepSolidity,",",keepAspectRatio)
	            if keepDims and keepSolidity and keepAspectRatio:
	                #cv2.drawContours(image, [approx], -1, (0,0,255), 4)
	                status = "Red Acquired"
	                self.detected = True
	                #get the center of the image
	                M = cv2.moments(approx)
	                (self.cX, self.cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	                length_side_pixels = self.w_act
	                obj_image_sensor = length_side_pixels / self.m
	                d_cam_image_estimate = targ["width"] * self.foc / obj_image_sensor
	                self.d_cam_image = d_cam_image_estimate
	                self.t_last_seen = time()
	                if self.debug_mode:
### This code slows everything down by ~10Hz
		                (startX, endX) = (int(self.cX - (w * .15)), int(self.cX + (w * .15)))
		                (startY, endY) = (int(self.cY - (h * .15)), int(self.cY + (h * .15)))
		    #             #draw target bounding box (red lines)
		                cv2.line(self.frame, (startX, self.cY), (endX, self.cY), (0,0,255),2)
		                cv2.line(self.frame, (self.cX, startY), (self.cX, endY), (0,0,255),2)
		    #             #indicate offset from center of image (blue lines)
		                cv2.line(self.frame, (self.c_x_image, self.c_y_image), (self.c_x_image, self.cY), (255,0,0),1)
		             	cv2.line(self.frame, (self.c_x_image, self.cY), (self.cX, self.cY), (255,0,0),1)
	                	break

	    if self.debug_mode:
	    	cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
	    #disable imshow for field tests
	    #cv2.imshow("Frame",image)
	    	self.out.write(self.frame)

	def detect_aruco_targets(self):
		"""Detects ARUCO targets."""
		self.detected = False
		status = "No target detected."
		gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
		num_boards_seen = 0
		outfile = 0
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
		if ids is not None:
			if self.debug_mode:
				self.frame = aruco.drawDetectedMarkers(self.frame, corners, ids)
			ids = [el[0] for el in ids]
			for b in range(len(self.targs)): #go through each board
				targ = self.targs[b]
				#obtain the corners and ids that pertain to this board
				c = [el for j,el in enumerate(corners) if ids[j] in targ.ids] 
				b_ids = [el for el in ids if el in targ.ids]				
				if b_ids == []:
					continue
				num_boards_seen += 1
				#only estimate pose one board at a time
				l,rvec,tvec = aruco.estimatePoseBoard(c,np.array(b_ids),targ.board,self.camera_matrix,self.dist_coeff)
				if self.debug_mode:
					frame = aruco.drawAxis(self.frame,self.camera_matrix,self.dist_coeff,rvec,tvec,self.targs[b].properties["size_square"])
				#project location of target back into image frame
				camera_points = cv2.projectPoints(np.array([(targ.properties["pos_rel"]).T]), 
		 			rvec, tvec, self.camera_matrix, self.dist_coeff)
		 		camera_points = np.array(camera_points[0][0][0])
		 		camera_points[0] = np.minimum(camera_points[0],self.horizontal_resolution)
		 		camera_points[1] = np.minimum(camera_points[1],self.vertical_resolution)
			 	camera_points[0] = np.maximum(camera_points[0],0)
			 	camera_points[1] = np.maximum(camera_points[1],0)
		 		targ.props_are_set = True #set a flag that we have seen this particular target
		 		targ.corners = c
			 	targ.r_mat = np.matrix(cv2.Rodrigues(rvec)[0])
			 	yaw = np.arctan2(targ.r_mat[1,0], targ.r_mat[0,0]) * 180 / math.pi
			 	targ.yaw = yaw - math.pi/2
			 	self.cop.set_last_recorded_yaw(yaw)
			 	#print("Current rotation: %.2f degrees."%(yaw))
			 	targ.t_vec = np.matrix(tvec)

			 	#obtain the position of the (0,0,0) point of the camera in the real world
			 	targ.position_camera = -targ.r_mat.T * targ.t_vec
				targ.d_cam_image = np.linalg.norm(targ.position_camera - (targ.properties["pos_rel"]).T)
				#store position of camera target
				targ.camera_frame_loc = camera_points 
				if np.linalg.norm(camera_points) <= 10e6 and self.debug_mode:
					## draw a lil circle
					cv2.circle(self.frame,(int(camera_points[0]),int(camera_points[1])),6,(255,0,0))
			
			camera_positions = np.array([0,0])
			d_cam_image = 0
			count = 0

			#do we have more than one position estimate?
			if num_boards_seen > 1: 
				#make sure we are confident about these position estimates
				best_targs = self.check_location_confidence()
				if best_targs is not None:
					#average the two closest targets
					for t in best_targs:
						camera_positions += t.camera_frame_loc
						d_cam_image += t.d_cam_image
						self.last_recorded_height_m = np.append(self.last_recorded_height_m, t.position_camera[2], axis=None)
						count += 1
				else:
					if self.debug_mode and outfile == 0:
						status = "Unconfident in our decision! Ignoring..."
						cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
						self.out.write(self.frame)
						outfile += 1
					return	
			elif num_boards_seen == 1: #check to make sure this single target recognition isn't preposterous
				t_found = [t for t in self.targs if t.props_are_set]
				t_found = t_found[0]

				#obtain a reasonable estimate of the cameras distance to the target
				#print(t_found.corners)
				corners = t_found.corners[0]
				#print(corners.shape)
				if((corners.shape)[0] == 1):
					corners = corners[0]
				length_side_pixels = np.linalg.norm(corners[0] - corners[1])
				obj_image_sensor = length_side_pixels / self.m
				d_cam_image_estimate = t_found.properties["size_square"] * self.foc / obj_image_sensor
				# max allowable discrepancy between drone height and distance from drone to target
				# not a great bound, but its the best I can think of 
				epsilon = d_cam_image_estimate * 2

				if t_found.d_cam_image > epsilon:
					if self.debug_mode and outfile == 0:
						status = "Unconfident in our decision! Ignoring... %.2f, %.2f, %.2f"%(d_cam_image_estimate, np.linalg.norm(t_found.properties["pos_rel"]), t_found.d_cam_image)
						cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
						self.out.write(self.frame)
						outfile += 1
					return	
				else: #its reasonable
					camera_positions = t_found.camera_frame_loc
					d_cam_image = t_found.d_cam_image
					self.last_recorded_height_m = np.append(self.last_recorded_height_m, t_found.position_camera[2], axis=None)
					count = 1
			if count >= 1:
				#if we detected at least 1 target
				camera_positions /= count #average
				#populate these variables so that they are passed to the copter commands
				self.cX = camera_positions[0]
				self.cY = camera_positions[1]
				if self.debug_mode:
					cv2.circle(self.frame,(int(self.cX),int(self.cY)),4,(0,255,0))
				self.last_recorded_height = np.sum(self.last_recorded_height_m)/count
				self.cop.set_last_recorded_height(self.last_recorded_height)
				self.d_cam_image = d_cam_image / count #average
				self.detected = True #indicate that we have detected at least one target
				#status = "Marker detected, x: %.2f, y: %.2f, d: %.2f, h: %.2f, count: %d"%(self.cX,self.cY, self.d_cam_image, self.last_recorded_height, count)
				status = "d: %.2f, h: %.2f"%(self.d_cam_image, self.last_recorded_height)
				if self.debug_mode and outfile == 0:
					cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),1)
					self.out.write(self.frame)
					outfile += 1
				self.t_last_seen = time()
				self.last_recorded_height_m = np.array([])

		if self.debug_mode and outfile == 0:
	    	## Display the resulting frame
			cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),1)
			self.out.write(self.frame)
			outfile += 1

	def check_location_confidence(self):
		"""Tries to find the most confident decision."""
		## not the best way of doing things, but since the number of targets is fairly small its not a big deal
		epsilon_pixels = .05 * self.horizontal_resolution #arbitrary confidence factor
		epsilon_meters = .08
		pixel_distances = []
		actual_distances = []
		num_observed = 0
		for ti in self.targs:
			if ti.props_are_set:
				for tj in self.targs:
					if tj.props_are_set: 
						pixel_dist = np.linalg.norm(tj.position_camera - ti.position_camera)
						actual_dist = np.abs(tj.d_cam_image - ti.d_cam_image)
						if pixel_dist == 0:
							pixel_dist = 10000 #ignore two of the same points
							actual_dist = 10000
						pixel_distances.append(pixel_dist)	
						actual_distances.append(actual_dist)
					else:
						pixel_distances.append(10000)
						actual_distances.append(10000)
			else:
				for _ in self.targs:
					pixel_distances.append(10000)
					actual_distances.append(10000)
		min_ind_pixel = np.argmin(pixel_distances)
		min_ind_actual = np.argmin(actual_distances)
		#min_ind is encoded in base (num_targets); decode it to find the closest two points
		best_guys = [self.targs[min_ind_pixel/len(self.targs)],self.targs[min_ind_pixel%len(self.targs)]]
		if pixel_distances[min_ind_pixel] > epsilon_pixels or actual_distances[min_ind_actual] > epsilon_meters:
			#measurements are not trustworthy, return nothing
			return None

		return best_guys

	def get_target(self,eyedee):
		for t in self.targs:
			if t.properties["ident"] == eyedee:
				return t

	def launch_copter(self, target_altitude):
		i = 0
		self.cop.arm_nogps()
		made_it = False
		while i < 230:# and not self.cop.indoor_mode:
			made_it = self.cop.takeoff_nogps(target_altitude)
			if made_it:
				break
			self.frame = self.vs.read()
			self.detect_aruco_targets()
			self.fps.update()
			i += 1

		if not made_it:
			print("Failed to reach target altitude...")

		self.set_resolution("low")
		self.detected = False

	def set_resolution(self, res, video_file = "output.avi"):
		"""Closes and re-opens all camera related variables to change resolutions on the fly."""
		print("Setting resolution to %s."%res)
		high_res = (1088,720)
		med_res = (640,480)
		low_res = (320,240)	
		if self.vs is not None:	
			self.teardown()
			sleep(1)

		if res == "high": #high
			self.camera_matrix = np.array([[8.4360221747968785e+02, 0., 544.], [0., 8.4385823040303683e+02, 360.],[0., 0.,
    		1.]])
			self.dist_coeff = np.array([1.7626446405747770e-01, -3.4120481004692560e-01,
		    	-2.1890672094602151e-03, -3.6706857342688248e-05,
		    	8.1488779271148601e-02])
			res = high_res
			self.cop.horizontal_fov = 76.884
			self.cop.vertical_fov = 51.714

		elif res == "med": #med
			self.camera_matrix = np.array([[4.9855533317091482e+02, 0., 320.], [0., 4.9967286973785622e+02, 240.],[0., 0.,
		1.]])
			self.dist_coeff = np.array([1.9695524980263868e-01, -4.7266256496392656e-01,
    			-2.8509501186610737e-03, -6.6742476969470338e-04,
    			2.9734384543609033e-01])
			res = med_res
			self.cop.horizontal_fov = 85.521
			self.cop.vertical_fov = 69.626

		elif res == "low": #low
			self.camera_matrix = np.array([[2.4848460687057266e+02, 0., 160.], [0., 2.4930955561049109e+02, 120.], [0., 0.,
    		1.]])
			self.dist_coeff = np.array([2.1646548043084851e-01, -6.2149098910402545e-01,
    			-1.9510859152085493e-03, -1.6281010642558004e-03,
   				5.5614584686671453e-01])
			res = low_res
			self.cop.horizontal_fov = 83.237
			self.cop.vertical_fov = 68.536
		
		self.vs = VideoStream(usePiCamera=True, resolution=res).start()
		sleep(.5)

		#set copter properties
		self.cop.horizontal_resolution = res[0]
		self.cop.vertical_resolution = res[1]
		#full camera fov
		self.cop.horizontal_fov_rad = self.cop.horizontal_fov * math.pi / 180
		self.cop.vertical_fov_rad = self.cop.vertical_fov * math.pi / 180

		frame = self.vs.read()
		self.horizontal_resolution = frame.shape[1]
		self.vertical_resolution = frame.shape[0]
		self.c_x_image = self.horizontal_resolution / 2
		self.c_y_image = self.vertical_resolution / 2
		self.frame = frame

		#for color target stuff
		f_x = self.camera_matrix[0][0]
		f_y = self.camera_matrix[1][1]
		self.m = (f_x + f_y) / (2 * self.foc)

		#file writing
		fpsL = 60
		fpsM = 125
		fpsH = 40
		
		fourcc = cv2.VideoWriter_fourcc(*'MJPG')
		if res == high_res:
			self.out = cv2.VideoWriter(video_file, fourcc,fpsH,(self.horizontal_resolution,self.vertical_resolution),True)
		elif res == med_res:
			self.out = cv2.VideoWriter(video_file, fourcc,fpsM,(self.horizontal_resolution,self.vertical_resolution),True)
		elif res == low_res:
			self.out = cv2.VideoWriter(video_file, fourcc,fpsL,(self.horizontal_resolution,self.vertical_resolution),True)

		self.fps = FPS().start()

	def need_target(self):
		"""Function to determine whether we should search for color targets."""
		# if we search for color targets, we are using cpu and potentially throwing the copter off the real target
		# can make this decision more complex if needded
		delta_t = time() - self.t_last_seen
		time_to_see = delta_t > .6 # arbitrary time threshold over which we should probably look for color targets
		
		return time_to_see

	def loop(self):
		# Read an image and run the image processing algorithm
		self.frame = self.vs.read()
		self.detect_aruco_targets()		
		if not self.detected and self.need_target():
			pass
			#self.detect_colors(self.color_targs) 
		if self.detected and self.d_cam_image > .2 :
			self.cop.send_land_message_angular(self.cX,self.cY,self.d_cam_image)
		self.frame = [] #reset frame
		for t in self.targs: #reset this flag
				t.props_are_set = False	
		self.fps.update()

	def teardown(self):
		#saves video, closes resources
		self.fps.stop()
		cv2.destroyAllWindows()
		self.out.release()
		self.vs.stop()


