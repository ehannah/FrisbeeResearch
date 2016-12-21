#Written by Paul Houston

import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

#the first attempt at a frisbee finder function. This one works, but not as well as the second
def findFrisbee(frame,window):
    #window is the size of the sweeping window
    imageDimensions = np.shape(frame)

    #the value of a white pixel is 255, so the average starts at 256 and is reassigned if a lower average is found
    minavg = 256
    #the default location is the top left corner of the video
    frisbeeLocation = (0,0)

    #loop over the frame moving the window one pixel at a time
    #The window cannot go outside of the frame, which is why the range has the window size subtracted frome it
    for ii in xrange(0,imageDimensions[0]-window[0]-1):
        for jj in xrange(0,imageDimensions[1]-window[1]-1):

            #find the average in the window with numpy's mean function
            avgPixel = np.mean(frame[ii:(ii+window[0]),jj:(jj+window[1])])

            #If the average in the window is less than the stored average, the stored average becomes the average in the window and frisbee location is reassigned
            #This iterates until the window has swept the entire frame, returning the location with the lowest average
            if minavg > avgPixel:
                minavg = avgPixel
                frisbeeLocation = (ii,jj)
    return frisbeeLocation

#this one is more accurate but relies on the filtering being robust
def findFrisbee2(frame):
    imageDimensions = np.shape(frame)

    #find the top and bottom of the box. The box starts out as the whole frame
    boxTop = 0
    boxTopFound = False
    boxBot = imageDimensions[0]-1

    #loop over each row of the frame
    for ii in xrange(0,imageDimensions[0]-1):
        #if any pixel in the row is black (value of zero), which is the color of the frisbee after thresholding, the top of the box is assigned to be that row
        #this only happens if the top of the box has not already been found        
        #this could be improved to be less sensitive with some sort of averaging instead of being triggered by one pixel
        if np.any(frame[ii,:]==0) and not boxTopFound:
            boxTop = ii
            boxTopFound = True
        #once the top of the box has been found, the next line of white (value of 255) that gets looped over is assigned to be the bottom of the box and the loop ends
        if boxTopFound and np.all(frame[ii,:] == 255):
            boxBot = ii
            break

    #find the sides of the box.
    boxLeft = 0
    boxLeftFound = False
    boxRight = imageDimensions[1]-1
    
    #same idea as the top and bottom finder
    #loop over each column of the frame
    for ii in xrange(0,imageDimensions[1]-1):
        #if any values between the top of the box and the bottom of the box are black, that row is assigned to be the left side of the box
        #only happens if the left side has not already been found
        #again, could be made less sensitive
        if np.any(frame[boxTop:boxBot,ii]==0) and not boxLeftFound:
            boxLeft = ii
            boxLeftFound = True
        #the next row of all white between the bottom and top of the box is assigned to be the right side and the loop ends
        if boxLeftFound and np.all(frame[boxTop:boxBot,ii]==255):
            boxRight = ii
            break
    #return the values in a tuple
    box = (boxTop,boxBot,boxLeft,boxRight)      
    return box

#This was a median filter that I made before realizing that OpenCV already has a function for it

#def median2Filter(frame):
#    imageArray = np.asarray(frame)
#    fig = plt.imshow(imageArray, cmap = cm.Greys_r)
#    plt.show()
#    dimensions = np.shape(imageArray)
#    print dimensions
#    newImageArray = np.zeros(dimensions)
#    for ii in xrange(2,dimensions[0]-3):
#        for jj in xrange(2,dimensions[1]-3):
#            pixelValues = ( imageArray[ii-1,jj],
#                            imageArray[ii,jj-1],
#                            imageArray[ii+1,jj],
#                            imageArray[ii,jj+1],
#                            imageArray[ii,jj],
#                            imageArray[ii-2,jj],
#                            imageArray[ii-1,jj-1],
#                            imageArray[ii,jj-2],
#                            imageArray[ii+1,jj-1],
#                            imageArray[ii+2,jj],
#                            imageArray[ii+1,jj+1],
#                            imageArray[ii,jj+2],
#                            imageArray[ii+1,jj-1])
#            newImageArray[ii,jj] = np.median(pixelValues)
#    fig = plt.imshow(newImageArray, cmap = cm.Greys_r)
#    plt.show()
#    return newImageArray
