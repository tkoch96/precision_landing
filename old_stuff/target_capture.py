import numpy as np
import cv2
import time

from imutils.video import VideoStream
from imutils.video import FPS


vs = VideoStream(usePiCamera=True, resolution=(320,240)).start()
time.sleep(2)
fps = FPS().start()
i=0
while True:
    i+=1
    print("Frame: %d"%i)
    frame = vs.read()
    image  = frame
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7,7), 0)
    edged = cv2.Canny(blurred, 50, 150)
    _, cnts, _ = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    status = "Target not Acquired, frame %d"%i

    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, .01 * peri, True)
        if len(approx) >= 4 and len(approx) <= 6:
            (x,y,w,h) = cv2.boundingRect(approx)
            aspectRatio = w /float(h)

            area = cv2.contourArea(c)
            hullArea = cv2.contourArea(cv2.convexHull(c))
            solidity = area / float(hullArea)

            keepDims = w > 25 and h > 25
            keepSolidity = solidity > .9
            keepAspectRatio = aspectRatio >= .5 and aspectRatio <= 1.5
            if keepDims and keepSolidity and keepAspectRatio:

                cv2.drawContours(image, [approx], -1, (0,0,255), 4)
                status = "Target(s) Acquired, Frame %d"%i
            
                M = cv2.moments(approx)
                (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                (startX, endX) = (int(cX - (w * .15)), int(cX + (w * .15)))
                (startY, endY) = (int(cY - (h * .15)), int(cY + (h * .15)))
                cv2.line(image, (startX, cY), (endX, cY), (0,0,255),3)
                cv2.line(image, (cX, startY), (cX, endY), (0,0,255),3)
    cv2.putText(image, status, (20,30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)
    cv2.imshow("Frame",image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
   
    fps.update()

fps.stop()
cv2.destroyAllWindows()
vs.stop()
