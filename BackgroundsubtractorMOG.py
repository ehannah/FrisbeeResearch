#Written by Paul Houston

import numpy as np
import cv2

#read in the video
cap=cv2.VideoCapture('MOGtest.mp4')

#prime the background subtractor algorithm
fgbg=cv2.createBackgroundSubtractorMOG2()

while(1):
    ret,frame=cap.read()
    #apply the algorithm
    fgmask=fgbg.apply(frame)
    #show the images
    cv2.imshow('frame',fgmask)
    k = cv2.waitKey(30) & 0xff
    if k==27:
        break

cap.release()
cv2.destroyAllWindows()
