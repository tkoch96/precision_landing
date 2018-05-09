import cv2, numpy as np
from time import time,sleep
from imutils.video import VideoStream, FPS
import target
import cv2.aruco as aruco

class seen_targ():
	"""Little helper class."""
	def __init__(self,targ):
		#generic properties
		self.properties = targ
		self.r_mat = None
		self.t_vec = None
		self.position = None
		self.corners = None
		self.camera_frame_loc = None
		self.d_cam_image = None
		self.props_are_set = False

class Image_Processor():
	def __init__(self, cop, t_start, debug_mode):
		self.cop = cop
		self.t_start = t_start
		self.debug_mode = debug_mode
		self.t_last_heading_update = t_start
		self.heading_update_period = 5000 #check every now and then to update heading

		#for the detection of the target
		self.detected = 0
		self.cX = 0
		self.cY = 0
		self.frame = []

		#video stream object
		high_res = (1088,720)
		med_res = (640,480)
		low_res = (320,240)		
		res = high_res
		#set these parameters in the copter
		self.cop.horizontal_resolution = res[0]
		self.cop.vertical_resolution = res[1]
		self.vs = VideoStream(usePiCamera=True, resolution=res).start()
		sleep(2)
		frame = self.vs.read()
		self.horizontal_resolution = frame.shape[1]
		self.vertical_resolution = frame.shape[0]
		self.c_x_image = self.horizontal_resolution / 2
		self.c_y_image = self.vertical_resolution / 2

		#aruco targets
		self.frame = frame
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		self.parameters =  aruco.DetectorParameters_create()
		self.res_x_high = 3280.0
		self.res_y_high = 2464.0
		ratio_x = frame.shape[0] * 1.0 / self.res_x_high
		ratio_y = frame.shape[1] * 1.0 /self.res_y_high
		# self.camera_matrix = np.array([[  ratio_x * 2.52687063e+03,   0.00000000e+00,   ratio_x * 1.61626478e+03],
		#        [  0.00000000e+00,   ratio_y * 2.53403716e+03,   ratio_y * 1.37235370e+03],
		#        [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
		# self.dist_coeff= np.array([ 0.13976421, -0.12782904,  0.02107314, -0.00745702 , -0.05526272])
		self.camera_matrix = np.array([[  ratio_x * 2.59314595e+03,   0.00000000e+00,   ratio_x * 1.64325821e+03],
       [  0.00000000e+00,   ratio_y * 2.59258565e+03,   ratio_y * 1.24476146e+03],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]])
		self.dist_coeff = np.array([[ 0.17606792, -0.16426664, -0.00093347, -0.00092262, -0.4304011 ]])
		self.foc = .003039 #focal length
		f_x = self.camera_matrix[0][0]
		f_y = self.camera_matrix[1][1]
		self.m = (f_x + f_y) / (2 * self.foc)
		
        
        #target object setups
		self.targs = []
		for t in target.aruco_targets:
			self.targs.append(seen_targ(t))

		#file writing
		fourcc = cv2.VideoWriter_fourcc(*'MJPG')
		self.out = cv2.VideoWriter('output.avi',fourcc,20,(self.horizontal_resolution,self.vertical_resolution),True)

		self.fps = FPS().start()

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

	    status = "Target not Acquired, timestamp: %s"%(time() - self.t_start)
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
	            keepSolidity = solidity > .9
	            keepAspectRatio = aspectRatio >= targ["aspect_ratio_min"] and aspectRatio <= targ["aspect_ratio_max"]
	            #print(keepDims,",",keepSolidity,",",keepAspectRatio)
	            if keepDims and keepSolidity and keepAspectRatio:
	                #cv2.drawContours(image, [approx], -1, (0,0,255), 4)
	                status = "Target(s) Acquired, timestamp: %s"%(time() - self.t_start)
	                self.detected = 1
	                #get the center of the image
	                M = cv2.moments(approx)
	                (self.cX, self.cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

### This code slows everything down by ~10Hz
	                (startX, endX) = (int(self.cX - (w * .15)), int(self.cX + (w * .15)))
	                (startY, endY) = (int(self.cY - (h * .15)), int(self.cY + (h * .15)))
	    #             #draw target bounding box (red lines)
	                cv2.line(self.frame, (startX, self.cY), (endX, self.cY), (0,0,255),2)
	                cv2.line(self.frame, (self.cX, startY), (self.cX, endY), (0,0,255),2)
	    #             #indicate offset from center of image (blue lines)
	                cv2.line(self.frame, (self.c_x_image, self.c_y_image), (self.c_x_image, self.cY), (255,0,0),1)
	                cv2.line(self.frame, (self.c_x_image, self.cY), (self.cX, self.cY), (255,0,0),1)

	    cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
	    #disable imshow for field tests
	    #cv2.imshow("Frame",image)
	    self.out.write(self.frame)

	def detect_aruco_targets(self):
		"""Detects ARUCO targets."""
		self.detected = False
		status = "No target detected."
		gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
		
	 	position_dict = {}
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
		if ids is not None:
			if self.debug_mode:
				self.frame = aruco.drawDetectedMarkers(self.frame, corners, ids)
			ids = [el[0] for el in ids]
		 	for i in range(len(ids)):
		 		eyedee = ids[i]
		 		targ = self.get_target(eyedee)
		 		targ.corners = corners[i]
		 		rvec,tvec,_ = aruco.estimatePoseSingleMarkers(corners[i],targ.properties["l_side"],self.camera_matrix,self.dist_coeff)
		 		if self.debug_mode:
			 		self.frame = aruco.drawAxis(self.frame,self.camera_matrix,self.dist_coeff,rvec,tvec,targ.properties["l_side"]/2)
			 	if eyedee in [t.properties["ident"] for t in self.targs]:
			 		#for each target that we recognize			 		
			 		targ.props_are_set = True #set a flag that we have seen this particular target
				 	targ.r_mat = np.matrix(cv2.Rodrigues(rvec)[0])
				 	targ.t_vec = np.matrix(tvec).T

			 		camera_points = cv2.projectPoints(np.array([(targ.properties["pos_rel"]).T]), 
			 			rvec, tvec, self.camera_matrix, self.dist_coeff)
			 		camera_points = np.array(camera_points[0][0][0])

				 	#obtain the position of the (0,0,0) point of the camera in the real world
				 	targ.position_camera = -targ.r_mat.T * targ.t_vec
					targ.d_cam_image = np.linalg.norm(targ.position_camera - (targ.properties["pos_rel"]).T)
					
					#store position of camera target
					targ.camera_frame_loc = camera_points 
					if np.linalg.norm(camera_points) > 10e6:
						print("Returned")
						return
					if self.debug_mode:
						## draw a lil circle
						cv2.circle(self.frame,(int(camera_points[0]),int(camera_points[1])),6,(255,0,0))
			
			camera_positions = np.array([0,0])
			d_cam_image = 0
			count = 0

			#do we have more than one position estimate?
			if len(ids) > 1: 
				#make sure we are confident about these position estimates
				best_targs = self.check_location_confidence()
				if best_targs is not None:
					#average the two closest targets
					for t in best_targs:
						camera_positions += t.camera_frame_loc
						d_cam_image += t.d_cam_image
						count += 1
				else:
					if self.debug_mode:
						status = "Unconfident in our decision! Ignoring..."
						cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
						self.out.write(self.frame)
					return	
			elif len(ids) == 1: #check to make sure this single target recognition isn't preposterous
				#use barometer as a rough guess
				

				t_found = [t for t in self.targs if t.props_are_set]
				t_found = t_found[0]

				#obtain a reasonable estimate of the cameras distance to the target
				corners = t_found.corners[0]
				length_side_pixels = np.linalg.norm(corners[0] - corners[1])
				obj_image_sensor = length_side_pixels / (self.horizontal_resolution * self.m)
				d_cam_image_estimate = t_found.properties["l_side"] * self.foc / obj_image_sensor
				# max allowable discrepancy between drone height and distance from drone to target
				# not a great bound, but its the best I can think of 
				epsilon = d_cam_image_estimate + np.linalg.norm(t_found.properties["pos_rel"])

				if t_found.d_cam_image > epsilon:
					if self.debug_mode:
						status = "Unconfident in our decision! Ignoring... %.2f, %.2f, %.2f"%(d_cam_image_estimate, np.linalg.norm(t_found.properties["pos_rel"]), t_found.d_cam_image)
						cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
						self.out.write(self.frame)
					return	
				else: #its reasonable
					camera_positions = t_found.camera_frame_loc
					d_cam_image = t_found.d_cam_image
					count = 1

			if count >= 1:
				#if we detected at least 1 target
				camera_positions /= count #average
				#populate these variables so that they are passed to the copter commands
				self.cX = camera_positions[0]
				self.cY = camera_positions[1]
				self.d_cam_image = d_cam_image / count #average
				self.detected = True #indicate that we have detected at least one target
				status = "Marker detected, x: %.2f, y: %.2f, d: %.2f"%(self.cX,self.cY, self.d_cam_image)

		if self.debug_mode:
		    ## Display the resulting frame
			cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
			self.out.write(self.frame)

	def check_location_confidence(self):
		"""Tries to find the most confident decision."""
		## not the best way of doing things, but since the number of targets is fairly small its not a big deal
		epsilon_pixels = .2 * self.horizontal_resolution #arbitrary confidence factor
		epsilon_meters = .5
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

	def detect_targets(self):
		for t in target.targets:
			if t["type"] == 'color':
				self.detect_colors(t)

	def get_target(self,eyedee):
		for t in self.targs:
			if t.properties["ident"] == eyedee:
				return t

	def loop(self):
		# Read an image and run the image processing algorithm
		self.frame = self.vs.read()
		self.detect_aruco_targets()		
		if self.detected:
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


