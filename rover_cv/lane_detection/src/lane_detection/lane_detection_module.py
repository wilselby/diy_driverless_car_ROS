#!/usr/bin/env python
# -*- coding: utf-8 -*-
# https://github.com/paramaggarwal/CarND-LaneLines-P1/blob/master/P1.ipynb
#https://github.com/NikolasEnt/Advanced-Lane-Lines/blob/master/LaneLine.ipynb

from __future__ import print_function
from __future__ import division
import roslib
import sys
import traceback
import rospy
import cv2
import numpy as np
import math
import logging
import socket
import threading
import time
import datetime
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

def limit(input, min, max):
    if input < min:
        input = min
    if input > max:
        input = max
    return input

def perp(a ) :
     b = np.empty_like(a)
     b[0] = -a[1]
     b[1] = a[0]
     return b

 # line segment a given by endpoints a1, a2
 # line segment b given by endpoints b1, b2
 # return 
def seg_intersect(a1,a2, b1,b2):
     da = a2-a1
     db = b2-b1
     dp = a1-b1
     dap = perp(da)
     denom = np.dot( dap, db)
     num = np.dot( dap, dp )
     return (num / denom.astype(float))*db + b1

def movingAverage(avg, new_sample, N=15):
     if (avg == 0):
         return new_sample
     avg -= avg / N;
     avg += new_sample / N;
     return avg
 
def drawQuad(image, points, color=[255, 0, 0], thickness=4):
        p1, p2, p3, p4 = points
        cv2.line(image, tuple(p1), tuple(p2), color, thickness)
        cv2.line(image, tuple(p2), tuple(p3), color, thickness)
        cv2.line(image, tuple(p3), tuple(p4), color, thickness)
        cv2.line(image, tuple(p4), tuple(p1), color, thickness)
     
def perspective_transform(image, corners, debug=False, xoffset=0):
    
     height, width = image.shape[0:2]
     output_size = height/2
     
     new_top_left=np.array([corners[0,0],0])
     new_top_right=np.array([corners[3,0],0])
     offset=[xoffset,0]    
     img_size = (image.shape[1], image.shape[0])
     src = np.float32([corners[0],corners[1],corners[2],corners[3]])
     dst = np.float32([corners[0]+offset,new_top_left+offset,new_top_right-offset ,corners[3]-offset]) 
    
     M = cv2.getPerspectiveTransform(src, dst)

     warped = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
     
     if debug:
         drawQuad(image, src, [255, 0, 0])
         drawQuad(warped, dst, [255, 255, 0])
         plt.imshow(image)
         plt.show()
         plt.imshow(warped)
         plt.show()
         
     return warped,  src,  dst

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)    
    
def binary_thresh( img,  boundaries,  filter):
    
     if filter == 'RGB':
         frame_to_thresh = img.copy()
     else:
         frame_to_thresh = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
     for (lower,  upper) in boundaries:
     
         # create numpy arrays from the boundaries
         lower = np.array(lower,  dtype = "uint8")
         upper = np.array(upper,  dtype = "uint8")
         
         # find the colors within the specified boundaries and apply the mask
         mask = cv2.inRange(frame_to_thresh,  lower,  upper)
         output = cv2.bitwise_and(frame_to_thresh, frame_to_thresh,  mask = mask)   #Returns an RGB image
     
     return mask
    
def HLS_sobel(img, s_thresh=(120, 255), sx_thresh=(20, 255),l_thresh=(40,255)):
    img = np.copy(img)
    
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
    #h_channel = hls[:,:,0]
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    # Sobel x
    # sobelx = abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255))
    # l_channel_col=np.dstack((l_channel,l_channel, l_channel))
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold saturation channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1

    # Threshold lightness
    l_binary = np.zeros_like(l_channel)
    l_binary[(l_channel >= l_thresh[0]) & (l_channel <= l_thresh[1])] = 1
    
    channels = 255*np.dstack(( l_binary, sxbinary, s_binary)).astype('uint8')        
    binary = np.zeros_like(sxbinary)
    binary[((l_binary == 1) & (s_binary == 1) | (sxbinary==1))] = 1
    binary = 255*np.dstack((binary,binary,binary)).astype('uint8')            
    return  binary,channels

def abs_thresh(img, thresh=(0, 255)):
    # 1) Create a mask of 1's where image is > thresh_min and < thresh_max
    binary = np.zeros_like(img)
    binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

    # 2) Return this mask as your binary_output image
    return binary

def fit_polynomials(image, binary_warped, debug=False):

    global ploty
    global fitx
    global fit
            
    if isinstance( binary_warped, int ) or binary_warped.shape[0] == 3:
         ploty = np.zeros(shape=(1, 1))
         fitx = np.zeros(shape=(1, 1))
         fit = None
         return ploty, fitx, fit
    
    # Assuming you have created a warped binary image called "binary_warped"
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[int(binary_warped.shape[0]/2):,:], axis=0)

    # Create an output image to draw on and  visualize the result
    out_img = np.copy(image)
    
    # Find the peak of the histogram
    midpoint = np.int(histogram.shape[0]/2)
    x_base = midpoint

    # Choose the number of sliding windows
    nwindows = 9

    # Set height of windows
    window_height = np.int(binary_warped.shape[0]/nwindows)    
    
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Current positions to be updated for each window
    x_current = x_base

    # Set the width of the windows +/- margin
    margin = 50

    # Set minimum number of pixels found to recenter window
    minpix = 10

    # Create empty lists to receive left and right lane pixel indices
    lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):

        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_x_low = x_current - margin
        win_x_high = x_current + margin

        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_x_low,win_y_low),(win_x_high,win_y_high),(0,255,0), 12) 

        # Identify the nonzero pixels in x and y within the window
        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

        # Append these indices to the list
        lane_inds.append(good_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_inds) > minpix:
            x_current = np.int(np.mean(nonzerox[good_inds]))

    # Concatenate the arrays of indices
    lane_inds = np.concatenate(lane_inds)

    # Extract line pixel positions
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds] 

    if x.shape[0] !=0 and y.shape[0] !=0:

        """
        # Fit a second order polynomial to each
        fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        fitx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
        """
        # Fit a first order polynomial to each
        fit = np.polyfit(y, x, 1)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        fitx = fit[0]*ploty + fit[1]
        
    else:
		ploty = np.zeros(shape=(1, 1))
		fitx = np.zeros(shape=(1, 1))
		fit = None

    if debug:
        out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 0, 0]
        plt.imshow(out_img)
        plt.plot(fitx, ploty, color='yellow')
        #plt.xlim(0, 1280)
        #plt.ylim(720, 0)
        plt.show()
    
    return ploty, fitx, fit
    
def fast_fit_polynomials(binary_warped, fit):
    # Assume you now have a new warped binary image 
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    #lane_inds = ((nonzerox > (fit[0]*(nonzeroy**2) + fit[1]*nonzeroy + fit[2] - margin)) & (nonzerox < (fit[0]*(nonzeroy**2) + fit[1]*nonzeroy + fit[2] + margin))) 
    lane_inds = ((nonzerox > (fit[0]*nonzeroy + fit[1] - margin)) & (nonzerox < fit[0]*nonzeroy + fit[1] + margin))
  
    # Again, extract  line pixel positions
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds] 
    
    if x.shape[0] !=0 and y.shape[0] !=0:

        """
        # Fit a second order polynomial
        fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        fitx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
        """
        # Fit a first order polynomial to each
        fit = np.polyfit(y, x, 1)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
        fitx = fit[0]*ploty + fit[1]
        
    else:
		ploty = np.zeros(shape=(1, 1))
		fitx = np.zeros(shape=(1, 1))
		fit = None
    
    return ploty, fitx, fit
    
def render_lane(image, corners, ploty, fitx, ):

    _,  src,  dst = perspective_transform(image, corners)
    Minv = cv2.getPerspectiveTransform(dst, src)

    # Create an image to draw the lines on
    warp_zero = np.zeros_like(image[:,:,0]).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # Recast the x and y points into usable format for cv2.fillPoly()
    pts = np.vstack((fitx,ploty)).astype(np.int32).T

    # Draw the lane onto the warped blank image
    #plt.plot(left_fitx, ploty, color='yellow')
    cv2.polylines(color_warp,  [pts],  False,  (0, 255, 0),  10)
    #cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    newwarp = cv2.warpPerspective(color_warp, Minv, (image.shape[1], image.shape[0])) 

    # Combine the result with the original image
    result = cv2.addWeighted(image, 1, newwarp, 0.3, 0)
    
    return result
    
def abs_thresh(img, thresh=(0, 255)):
    # 1) Create a mask of 1's where image is > thresh_min and < thresh_max
    binary = np.zeros_like(img)
    binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

    # 2) Return this mask as your binary_output image
    return binary
    
def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image
    
def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.

    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    return lines
    
def draw_lane_lines(img, lines, color=[255, 0, 0], thickness=2):
     """
     NOTE: this is the function you might want to use as a starting point once you want to 
     average/extrapolate the line segments you detect to map out the full
     extent of the lane (going from the result shown in raw-lines-example.mp4
     to that shown in P1_example.mp4).  
     
     Think about things like separating line segments by their 
     slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
     line vs. the right line.  Then, you can average the position of each of 
     the lines and extrapolate to the top and bottom of the lane.
     
     This function draws `lines` with `color` and `thickness`.    
     Lines are drawn on the image inplace (mutates the image).
     If you want to make the lines semi-transparent, think about combining
     this function with the weighted_img() function below
     """
     # state variables to keep track of most dominant segment
     largestLeftLineSize = 0
     largestRightLineSize = 0
     largestLeftLine = (0,0,0,0)
     largestRightLine = (0,0,0,0)          
     line_img = np.zeros_like(img)
     inputImg = np.copy(img)
     intersectionPoint = (0, 0)
     
     avgLeft = (0,0,0,0)
     avgRight = (0,0,0,0)
     
     if lines is None:
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(line_img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(line_img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
        return inputImg,  intersectionPoint
        
     if lines is not None:
          for line in lines:
            for x1,y1,x2,y2 in line:
                size = float(math.hypot(x2 - x1, y2 - y1))
                slope = float((y2-y1)/(x2-x1))                
                # Filter slope based on incline and
                # find the most dominent segment based on length
                if (slope > 0.1): #right
                    if (size > largestRightLineSize):
                        largestRightLine = (x1, y1, x2, y2)                    
                    cv2.line(line_img, (x1, y1), (x2, y2), (255,0, 0),2) #Show every line found
                elif (slope < -0.1): #left
                    if (size > largestLeftLineSize):
                        largestLeftLine = (x1, y1, x2, y2)
                    cv2.line(line_img, (x1, y1), (x2, y2), (255,0,0),2)    #Show every line found
                 
     # Show largest line found on either side
     #cv2.line(inputImg, (largestRightLine[0], largestRightLine[1]), (largestRightLine[2], largestRightLine[3]), (255,0,0),8)
     #cv2.line(inputImg, (largestLeftLine[0], largestLeftLine[1]), (largestLeftLine[2], largestLeftLine[3]), (255,0,0),8) 
 
     # Define an imaginary horizontal line in the center of the screen
     # and at the bottom of the image, to extrapolate determined segment
     imgHeight, imgWidth = (inputImg.shape[0], inputImg.shape[1])
     upLinePoint1 = np.array( [0, int(imgHeight - (imgHeight/2))] )
     upLinePoint2 = np.array( [int(imgWidth), int(imgHeight - (imgHeight/2))] )
     downLinePoint1 = np.array( [0, int(imgHeight)] )
     downLinePoint2 = np.array( [int(imgWidth), int(imgHeight)] )
      
     # Find the intersection of dominant lane with an imaginary horizontal line
     # in the middle of the image and at the bottom of the image.
     p3 = np.array( [largestLeftLine[0], largestLeftLine[1]] )
     p4 = np.array( [largestLeftLine[2], largestLeftLine[3]] )
     upLeftPoint = seg_intersect(upLinePoint1,upLinePoint2, p3,p4)
     downLeftPoint = seg_intersect(downLinePoint1,downLinePoint2, p3,p4)
     
     if (math.isnan(upLeftPoint[0]) or math.isnan(downLeftPoint[0])):
         avgx1, avgy1, avgx2, avgy2 = avgLeft
         #cv2.line(inputImg, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
         avgx1, avgy1, avgx2, avgy2 = avgRight
         #cv2.line(inputImg, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
     else:
         cv2.line(line_img, (int(upLeftPoint[0]), int(upLeftPoint[1])), (int(downLeftPoint[0]), int(downLeftPoint[1])), [0, 0, 255], 8) #draw left line
     
     # Calculate the average position of detected left lane over multiple video frames and draw
     if (math.isnan(upLeftPoint[0])== False and math.isnan(downLeftPoint[0]) == False):
         avgx1, avgy1, avgx2, avgy2 = avgLeft
         avgLeft = (movingAverage(avgx1, upLeftPoint[0]), movingAverage(avgy1, upLeftPoint[1]), movingAverage(avgx2, downLeftPoint[0]), movingAverage(avgy2, downLeftPoint[1]))
         avgx1, avgy1, avgx2, avgy2 = avgLeft
         cv2.line(inputImg, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
      
     # Find the intersection of dominant lane with an imaginary horizontal line
     # in the middle of the image and at the bottom of the image.
     p5 = np.array( [largestRightLine[0], largestRightLine[1]] )
     p6 = np.array( [largestRightLine[2], largestRightLine[3]] )
     upRightPoint = seg_intersect(upLinePoint1,upLinePoint2, p5,p6)
     downRightPoint = seg_intersect(downLinePoint1,downLinePoint2, p5,p6)
     if (math.isnan(upRightPoint[0]) or math.isnan(downRightPoint[0])):
         avgx1, avgy1, avgx2, avgy2 = avgLeft
         #cv2.line(inputImg, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
         avgx1, avgy1, avgx2, avgy2 = avgRight
         #cv2.line(inputImg, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
     else:
         cv2.line(line_img, (int(upRightPoint[0]), int(upRightPoint[1])), (int(downRightPoint[0]), int(downRightPoint[1])), [0, 0, 255], 8) #draw left line
      
      # Calculate the average position of detected right lane over multiple video frames and draw
     if (math.isnan(upRightPoint[0])== False and math.isnan(downRightPoint[0]) == False):
         avgx1, avgy1, avgx2, avgy2 = avgRight
         avgRight = (movingAverage(avgx1, upRightPoint[0]), movingAverage(avgy1, upRightPoint[1]), movingAverage(avgx2, downRightPoint[0]), movingAverage(avgy2, downRightPoint[1]))
         avgx1, avgy1, avgx2, avgy2 = avgRight         
         cv2.line(inputImg, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
      
     # Calculate intersection of detected lane lines
     if avgLeft == (0, 0, 0, 0) and avgRight != (0, 0, 0, 0):
         ar1 = np.array( [avgRight[0],  avgRight[1]] )
         ar2 = np.array( [avgRight[2],  avgRight[3]] )
         intersectionPoint = seg_intersect(ar1, ar2, upLinePoint1, upLinePoint2)
     if avgLeft != (0, 0, 0, 0) and avgRight == (0, 0, 0, 0):
         al1 = np.array( [avgLeft[0],  avgLeft[1]] )
         al2 = np.array( [avgLeft[2],  avgLeft[3]] )
         intersectionPoint = seg_intersect(al1, al2, upLinePoint1, upLinePoint2)
     if avgLeft != (0, 0, 0, 0) and avgRight != (0, 0, 0, 0):
         al1 = np.array( [avgLeft[0],  avgLeft[1]] )
         al2 = np.array( [avgLeft[2],  avgLeft[3]] )
         ar1 = np.array( [avgRight[0],  avgRight[1]] )
         ar2 = np.array( [avgRight[2],  avgRight[3]] )
         intersectionPoint = seg_intersect(al1, al2, ar1, ar2)

     if (math.isnan(intersectionPoint[0])== False and math.isnan(intersectionPoint[1]) == False): 
         cv2.circle(inputImg, (int(intersectionPoint[0]), int(intersectionPoint[1])), 12, (0, 255, 0), -1)
     
     return inputImg,  intersectionPoint
