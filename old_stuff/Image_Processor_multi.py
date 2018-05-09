import cv2, time,  numpy as np
import target
import cv2.aruco as aruco
from imutils.video import VideoStream, FPS

class seen_targ():
	"""Little helper class."""
	def __init__(self,targ):
		#generic properties
		self.properties = targ
		self.r_mat = None
		self.t_vec = None
		self.position = None
		self.camera_frame_loc = None
		self.d_cam_image = None
		self.props_are_set = False

class Image_Processor():
	"""Image processing task - performs the grunt work."""
	def __init__(self, frame):
		#for the detection of the target
		self.detected = False
		self.cX = 0
		self.cY = 0
		self.w_act = 0
		self.h_act = 0
		self.d_cam_image = 0
		self.num_jobs = 0

		#aruco targets
		self.frame = frame
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		self.parameters =  aruco.DetectorParameters_create()
		ratio_x = frame.shape[0] * 1.0 / 3280.0
		ratio_y = frame.shape[1] * 1.0 /2464.0
		self.camera_matrix = np.array([[ratio_x * 2593.763617, 0.00000000e+00, ratio_x * 1647.91014],
         [  0.00000000e+00, ratio_y * 2596.209383, ratio_y * 1271.02745], 
         [  0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
		self.dist_coeff= np.array([ 0.16660866, 0.26703296, 0.00303930, 0.00639334, -2.17691524])
        
        #target object setups
		self.targs = []
		for t in target.aruco_targets:
			self.targs.append(seen_targ(t))

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

	    status = "Target not Acquired, timestamp: %s"%(time.time() - self.t_start)
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
	                status = "Target(s) Acquired, timestamp: %s"%(time.time() - self.t_start)
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

	def __call__(self):
		#turn multiprocessing on 

		print("Trying to detect a target in a new frame.")
		"""Detects ARUCO targets."""
		self.detected = False
		gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
		status = "No Marker Detected"
		print(gray)
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
		
		print("Got here0.")
		if ids is not None:
			#self.frame = aruco.drawDetectedMarkers(self.frame, corners, ids)
		 	rvec,tvec,_ = aruco.estimatePoseSingleMarkers(corners,.203,self.camera_matrix,self.dist_coeff)
		 	for i in range(len(ids)):
			 	#self.frame =aruco.drawAxis(self.frame,self.camera_matrix,self.dist_coeff,rvec[i],tvec[i],.1)
			 	eyedee = ids[i][0]
			 	if eyedee in [t.properties["ident"] for t in self.targs]:
			 		#for each target that we recognize
			 		targ = self.get_target(eyedee)
			 		targ.props_are_set = True #set a flag that we have seen this particular target
				 	targ.r_mat = np.matrix(cv2.Rodrigues(rvec[i])[0])
				 	targ.t_vec = np.matrix(tvec[i]).T

			 		camera_points = cv2.projectPoints(np.array([(targ.properties["pos_rel"]).T]), 
			 			rvec[i], tvec[i], self.camera_matrix, self.dist_coeff)
			 		camera_points = np.array(camera_points[0][0][0])

				 	#obtain the position of the (0,0,0) point of the camera in the real world
				 	targ.position_camera = -targ.r_mat.T * targ.t_vec
					targ.d_cam_image = np.linalg.norm(targ.position_camera + (targ.properties["pos_rel"]).T)
					
					#store position of camera target
					targ.camera_frame_loc = camera_points 

					## draw a lil circle
					status = "Marker detected, x: %.2f, y: %.2f"%(camera_points[0],camera_points[1])
					#print("Distance to target %.2f"%targ.d_cam_image)
					#cv2.circle(self.frame,(int(camera_points[0]),int(camera_points[1])),4,(255,0,0))
		print("Got here1.")
	    ## Display the resulting frame
		#cv2.putText(self.frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
		#self.out.write(self.frame)

		camera_positions = np.array([0,0])
		d_cam_image = 0
		count = 0
		print("Got here3.")

		#somehow fuse the measurements from each target (average for now)
		for t in self.targs:
			if t.props_are_set:
				camera_positions += t.camera_frame_loc
				d_cam_image += t.d_cam_image
				count += 1
				t.props_are_set = False #reset flag
		if count >= 1:
			#if we detected at least 1 target
			camera_positions /= count
			#populate these variables so that they are passed to the copter commands
			self.cX = camera_positions[0]
			self.cY = camera_positions[1]
			self.d_cam_image = d_cam_image / count
			self.detected = True #indicate that we have detected at least one target
		print("Got here.")
		return [self.detected, self.cX, self.cY, self.d_cam_image]

	def get_target(self, eyedee):
		"""Retrieves a target in our list of targets with the corresponding id."""
		for t in self.targs:
			if eyedee == t.properties["ident"]:
				return t

