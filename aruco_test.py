import numpy as np
import cv2,time
import cv2.aruco as aruco
from imutils.video import VideoStream, FPS



low_res = (320,240)
high_res = (1088,720)
res = high_res

ratio_x = 1#res[0] * 1.0 / 3280.0
ratio_y = 1#res[1] * 1.0 / 2464.0

camera_matrix = np.array([[8.4360221747968785e+02, 0., 544.], [0., 8.4385823040303683e+02, 360.],[0., 0.,
    		1.]])
# dist_coeff = np.array([ 8.6177116439663443e-02, 6.4703420443218085e-01,
#     -2.2810630345485527e-03, -9.6791588364700742e-04,
#     -3.7166136619000865e+00])

dist_coeff = np.array([1.7626446405747770e-01, -3.4120481004692560e-01,
		    	-2.1890672094602151e-03, -3.6706857342688248e-05,
		    	8.1488779271148601e-02])
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi',fourcc,20,(res[0],res[1]),True)

def create_marker(ident=2):

	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	print(aruco_dict)
	# second parameter is id number
	# last parameter is total image size
	img = aruco.drawMarker(aruco_dict, ident, 700)
	cv2.imwrite("test_marker%d.jpg"%ident, img)

	#cv2.imshow('frame',img)
	#cv2.waitKey(0)
	#cv2.destroyAllWindows()


def detect_markers():
 	
	vs = VideoStream(usePiCamera=True, resolution=res).start() 
	fps = FPS().start()
	time.sleep(1)
	freq = 0
	i=0
	target_x = []
	target_y = []
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	while(i<50):
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
			frame = aruco.drawDetectedMarkers(frame, corners, ids)
		 	#rvec,tvec,_ = aruco.estimatePoseSingleMarkers(corners,.2,camera_matrix,dist_coeff)
		 	rvec,tvec = aruco.estimatePoseBoard(corners,ids,board,camera_matrix,dist_coeff)
		 	for i in range(len(ids)):
			 	frame = aruco.drawAxis(frame,camera_matrix,dist_coeff,rvec[i],tvec[i],.1)
			 	rmat = cv2.Rodrigues(rvec[i])[0]
			 	position_camera = -np.matrix(rmat).T * np.matrix(tvec[i]).T
			 	position_dict[ids[i][0]] = position_camera
			 	status = ("x: %.2f, y: %.2f, z: %.2f")%(position_camera[0],position_camera[1],position_camera[2])
	    # Display the resulting frame
		cv2.putText(frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
		out.write(frame)
		
		i += 1

		freq = .7*freq + .3 * (1/(time.time()-tf))
		fps.update()
		print("Operating frequency: %.2f Hz"%freq)
 
	# When everything done, release the capture
	out.release()
	vs.stop()
	cv2.destroyAllWindows()

def create_board(start_id):
	num_x = 1
	num_y = 2
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	board = aruco.GridBoard_create(num_x,num_y,1,.2,dictionary,start_id)
	img = board.draw((800*num_x,800*num_y))
	cv2.imwrite('boards/aruco_board%d.jpg'%start_id,img)

def detect_board():
	num_x = 2
	num_y = 2
	dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
	board = aruco.GridBoard_create(num_x,num_y,.0932,.01869,dictionary)
	vs = VideoStream(usePiCamera=True, resolution=res).start() 
	fps = FPS().start()
	time.sleep(1)
	freq = 0
	i=0
	target_x = []
	target_y = []
	while(i<100):
		tf = time.time()
	    # Capture frame-by-frame
		frame = vs.read()
	    # Our operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		parameters =  aruco.DetectorParameters_create()
	 	# print(parameters)
		status = "No Marker Detected"
		position_dict = {}
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=parameters)
		if ids is not None:
			frame = aruco.drawDetectedMarkers(frame, corners, ids)
			l,rvec,tvec = aruco.estimatePoseBoard(corners,ids,board,camera_matrix,dist_coeff)
			print(rvec)
			print(tvec)
			frame = aruco.drawAxis(frame,camera_matrix,dist_coeff,rvec,tvec,.1)
			camera_points = cv2.projectPoints(np.array([np.array([0,0,1.0]).T]), 
	 			rvec, tvec, camera_matrix, dist_coeff)
	 		camera_points = np.array(camera_points[0][0][0])
	 		status = "Target detect, x: %.2f, y: %.2f"%(camera_points[0],camera_points[1])
	 		if np.linalg.norm(camera_points) <= 10e6:
		 		cv2.circle(frame,(int(camera_points[0]),int(camera_points[1])),6,(255,0,0))


		# Display the resulting frame
		cv2.putText(frame, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
		out.write(frame)
		
		i += 1

		freq = .7*freq + .3 * (1/(time.time()-tf))
		fps.update()
		print("Operating frequency: %.2f Hz"%freq)
 
	# When everything done, release the capture
	out.release()
	vs.stop()
	cv2.destroyAllWindows()


def main():
	ids = [104,106,108,112]
	for eyedee in ids:
		create_board(eyedee)
	# s_ids = [120]
	# for s_id in s_ids:
	#	create_board(start_id = s_id)
	detect_board()

if __name__ == "__main__":
	main()

