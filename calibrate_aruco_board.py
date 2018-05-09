import time
import cv2
import cv2.aruco as aruco
import numpy as np
import glob

img_list = glob.glob('./chessboard_big/chessboard*.jpg')

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
num_x = 8
num_y = 5
params = cv2.aruco.DetectorParameters_create()
params.cornerRefinementMaxIterations = 0
board = cv2.aruco.CharucoBoard_create(num_x,num_y,0.08834,0.045,dictionary)
img = board.draw((400*num_x,400*num_y))

#Dump the calibration board to a file
cv2.imwrite('charuco.jpg',img)


allCorners = []
allIds = []
decimator = 0
for fname in img_list:
    print("Looking at file %s"%fname)
    frame = cv2.imread(fname)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary,  parameters=params)

    if len(res[0])>0:
	print("Found something!")
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])
	#write this cool thing to file
	cv2.imwrite('chessboard_found/chessboard%d.jpg'%decimator,gray)
    decimator += 1
imsize = gray.shape
print('Calibrating camera')
cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
print(cal)
