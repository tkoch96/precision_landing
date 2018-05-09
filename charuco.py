import numpy as np
import cv2,time
import cv2.aruco as aruco
from imutils.video import VideoStream, FPS



low_res = (320,240)
high_res = (1088,720)
res = low_res

ratio_x = res[0] * 1.0 / 3280.0
ratio_y = res[1] * 1.0 /2464.0
camera_matrix = np.array([[ratio_x * 2577.73797, 0.00000000e+00, ratio_x * 1697.34201],
		 [  0.00000000e+00, ratio_y * 2576.54763, ratio_y * 1263.96547], 
		 [  0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
dist_coeff= np.array([ 0.15510865, -0.07695723,  0.02155089, -0.00386623, -0.11443029])
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi',fourcc,20,(res[0],res[1]),True)

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
num_x = 8
num_y = 5
board = cv2.aruco.CharucoBoard_create(num_x,num_y,.0254,.0159,dictionary)

def create_board(ident=2):

	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	print(aruco_dict)
	# second parameter is id number
	# last parameter is total image size
	img = aruco.drawMarker(aruco_dict, ident, 700)
	cv2.imwrite("test_marker.jpg", img)

	cv2.imshow('frame',img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


def detect_board():
 	
	vs = VideoStream(usePiCamera=True, resolution=res).start() 
	fps = FPS().start()
	time.sleep(1)
	freq = 0
	i=0
	target_x = []
	target_y = []
	while(i<5000):
		tf = time.time()
	    # Capture frame-by-frame
		frame = vs.read()
	    # Our operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		parameters =  aruco.DetectorParameters_create()
	 	#print(parameters))
		status = "No Marker Detected"
		position_dict = {}
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		if ids is not None:
			 ret, ch_corners, ch_ids = aruco.interpolateCornersCharuco(corners, ids, gray, board )
			 
			 # if there are enough corners to get a reasonable result
			 if( ret > 5 ):
				 aruco.drawDetectedCornersCharuco(frame,ch_corners,ch_ids,(0,0,255) )
				 retval, rvec, tvec = aruco.estimatePoseCharucoBoard( ch_corners, ch_ids, board, camera_matrix, dist_coeff )
				# if a pose could be estimated
				 if( retval ) :
				 	frame = aruco.drawAxis(frame,camera_matrix,dist_coeff,rvec,tvec,0.1)
					rmat = cv2.Rodrigues(rvec)[0]

				 	position_camera = -np.matrix(rmat).T * np.matrix(tvec)
				 	position_dict[ids[0][0]] = position_camera
				 	status = ("x: %.2f, y: %.2f, z: %.2f")%(position_camera[0],position_camera[1],position_camera[2])
	    # Display the resulting frame
		cv2.putText(frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
		out.write(frame)
		#cv2.imshow('frame',gray)
		#if cv2.waitKey(1) & 0xFF == ord('q'):
	#		break
		i += 1

		freq = .7*freq + .3 * (1/(time.time()-tf))
		fps.update()
		#print("Operating frequency: %.2f Hz"%freq)
 
	# When everything done, release the capture
	out.release()
	vs.stop()
	cv2.destroyAllWindows()

def main():
	#create_board(4)
	detect_board()

if __name__ == "__main__":
	main()

