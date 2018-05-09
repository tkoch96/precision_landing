import math, time
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import cv2
import matplotlib.pyplot as plt

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


#video stream object
#framerate is about 5fps at this resolution
vs = VideoStream(usePiCamera=True, resolution=(640,480)).start()
time.sleep(2)
frame = vs.read()
horizontal_resolution = frame.shape[1]
vertical_resolution = frame.shape[0]
c_x_image = horizontal_resolution / 2
c_y_image = vertical_resolution / 2


#file writing
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi',fourcc,20,(horizontal_resolution,vertical_resolution),True)
out2 = cv2.VideoWriter('output2.avi',fourcc,20,(horizontal_resolution,vertical_resolution),True)
print("Start timer.")
t_start = time.time()

def detect_target(image):
	sens = 50
	white_higher = np.array([180,sens,255])
	white_lower = np.array([0,0,255-sens])
	red_higher = np.array([180,255,255])
	red_lower = np.array([130,sens,sens])
	blue_higher = np.array([129,255,255])
	blue_lower = np.array([90,sens,sens])
	green_higher = np.array([89,255,255])
	green_lower = np.array([0,sens,sens])	

	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	# construct a mask for the color "white", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, white_lower, white_higher)
	mask2 = cv2.inRange(hsv, red_lower, red_higher)
	mask3 = cv2.inRange(hsv, blue_lower, blue_higher)
	mask4 = cv2.inRange(hsv, green_lower, green_higher)
	(h,w) = image.shape[:2]

	#zeros = np.zeros((h,w), dtype="uint8")
	#R = cv2.merge([zeros, zeros, mask])
	#out2.write(R)

	#erode and dilate
	mask = cv2.erode(mask, None, iterations=1)
	mask = cv2.dilate(mask, None, iterations=1)
	mask2 = cv2.erode(mask2, None, iterations=1)
	mask2 = cv2.dilate(mask2, None, iterations=1)
	mask3 = cv2.erode(mask3, None, iterations=1)
	mask3 = cv2.dilate(mask3, None, iterations=1)
	mask4 = cv2.erode(mask4, None, iterations=1)
	mask4 = cv2.dilate(mask4, None, iterations=1)

	# find contours in the mask and initialize the current
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	cnts4 = cv2.findContours(mask4.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]	
	center = None

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
			#print(keepDims,",",keepSolidity,",",keepAspectRatio)
	    		if keepDims and keepSolidity and keepAspectRatio:
				cv2.drawContours(image, [approx], -1, (0,0,255), 4)
				status = "Target(s) Acquired, timestamp: %s"%(time.time() - t_start)
				detected = 1
				#get the center of the image
				M = cv2.moments(approx)
				(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				(startX, endX) = (int(cX - (w * .15)), int(cX + (w * .15)))
				(startY, endY) = (int(cY - (h * .15)), int(cY + (h * .15)))
				#draw target bounding box (red lines)
				cv2.line(image, (startX, cY), (endX, cY), (255,0,255),2)
				cv2.line(image, (cX, startY), (cX, endY), (255,0,255),2)
				#indicate offset from center of image (blue lines)
				cv2.line(image, (c_x_image, c_y_image), (c_x_image, cY), (255,0,0),1)
				cv2.line(image, (c_x_image, cY), (cX, cY), (255,0,0),1)
				#print("Aspect Ratio: %.4f"%aspectRatio)
	
	for c in cnts2:
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
			#print(keepDims,",",keepSolidity,",",keepAspectRatio)
	    		if keepDims and keepSolidity and keepAspectRatio:
				cv2.drawContours(image, [approx], -1, (0,0,255), 4)
				status = "Target(s) Acquired, timestamp: %s"%(time.time() - t_start)
				detected = 1
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
				
	for c in cnts3:
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
			#print(keepDims,",",keepSolidity,",",keepAspectRatio)
	    		if keepDims and keepSolidity and keepAspectRatio:
				cv2.drawContours(image, [approx], -1, (0,0,255), 4)
				status = "Target(s) Acquired, timestamp: %s"%(time.time() - t_start)
				detected = 1
				#get the center of the image
				M = cv2.moments(approx)
				(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				(startX, endX) = (int(cX - (w * .15)), int(cX + (w * .15)))
				(startY, endY) = (int(cY - (h * .15)), int(cY + (h * .15)))
				#draw target bounding box (red lines)
				cv2.line(image, (startX, cY), (endX, cY), (0,255,255),2)
				cv2.line(image, (cX, startY), (cX, endY), (0,255,255),2)
				#indicate offset from center of image (blue lines)
				cv2.line(image, (c_x_image, c_y_image), (c_x_image, cY), (255,0,0),1)
				cv2.line(image, (c_x_image, cY), (cX, cY), (255,0,0),1)
				#print("Aspect Ratio: %.4f"%aspectRatio)
				
	for c in cnts4:
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
			#print(keepDims,",",keepSolidity,",",keepAspectRatio)
	    		if keepDims and keepSolidity and keepAspectRatio:
				cv2.drawContours(image, [approx], -1, (0,0,255), 4)
				status = "Target(s) Acquired, timestamp: %s"%(time.time() - t_start)
				detected = 1
				#get the center of the image
				M = cv2.moments(approx)
				(cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

				(startX, endX) = (int(cX - (w * .15)), int(cX + (w * .15)))
				(startY, endY) = (int(cY - (h * .15)), int(cY + (h * .15)))
				#draw target bounding box (red lines)
				cv2.line(image, (startX, cY), (endX, cY), (255,255,255),2)
				cv2.line(image, (cX, startY), (cX, endY), (255,255,255),2)
				#indicate offset from center of image (blue lines)
				cv2.line(image, (c_x_image, c_y_image), (c_x_image, cY), (255,0,0),1)
				cv2.line(image, (c_x_image, cY), (cX, cY), (255,0,0),1)
				#print("Aspect Ratio: %.4f"%aspectRatio)
				
	cv2.putText(image, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
	#disable imshow for field tests
	#cv2.imshow("Frame",image)
	out.write(image)
def main():
	i=0
	print("Starting...")
	while(i<120):
		tf = time.time()
		frame = vs.read()
		detect_target(frame)
	    	key = cv2.waitKey(1) & 0xFF 
	    	if key == ord("q"):
            		break
		
		print("Operating frequency, %.2f Hz"%(1/(time.time()-tf)))
		i=i+1
	cv2.destroyAllWindows()
	out.release()
	out2.release()
if __name__ == "__main__":
	main()
