#Written by Paul Houston 

import numpy as np
import cv2
import image_functions as imfun
import time
#read in the video
cap=cv2.VideoCapture('run1cam2cut.mp4')

#prime the background subtractor algorithm
fgbg=cv2.createBackgroundSubtractorMOG2()

#set up the video saving mechanism
fourcc=cv2.VideoWriter_fourcc(*'DIVX')
out=cv2.VideoWriter('process1.mp4',fourcc,25.0,(720,480),1)

# read the first frame. Frame is an array that represents a single frame and ret is a variable that indicates whether there is more to the video
ret,frame=cap.read()

# while there are frames to read, process them
while(ret):
    
    

    #apply the background subtraction algorithm
    fgmask=fgbg.apply(frame)

    #apply a median filter to remove noise
    fgmask=cv2.medianBlur(fgmask,7)    

    # apply a threshold. This uses a different ret variable, ret1, because the entire operation does not end when the thresholding ends
    ret1,fgmask = cv2.threshold(fgmask, 127, 255, cv2.THRESH_BINARY_INV)

    #apply a median filter again to remove the last bits of noise
    fgmask=cv2.medianBlur(fgmask,7)

    #find the frisbee with the finder function. see image_functions.py
    box = imfun.findFrisbee2(fgmask)

    #draw a box with what the function returns and print the boundaries
    linesize = 2
    frame[box[0]:box[1],box[2]:(box[2]+linesize),:] = (0,0,255)
    frame[box[0]:box[1],box[3]:(box[3]+linesize),:] = (0,0,255)
    frame[box[0]:(box[0]+linesize),box[2]:box[3],:] = (0,0,255)
    frame[box[1]:(box[1]+linesize),box[2]:box[3],:] = (0,0,255)
    print box

    #show the images
    cv2.imshow('result',frame)
    
    #write the processed frame
    out.write(frame)

    # wait 0.5 seconds before reading the next frame
    cv2.waitKey(5)

    # read the next frame
    ret,frame=cap.read()
    

#close everything
cap.release()
cv2.destroyAllWindows()
out.release()
