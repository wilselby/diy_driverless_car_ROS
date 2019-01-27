#!/usr/bin/env python
# -*- coding: utf-8 -*-
#https://github.com/paramaggarwal/CarND-Advanced-Lane-Lines/blob/master/Notebook.ipynb
from __future__ import print_function
from __future__ import division
import sys
import traceback
import rospy
import numpy as np
import cv2
import pickle
import glob
import time
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class camera_calibarion(object):
    def __init__(self):
            
      """ROS Subscriptions """
      self.image_pub = rospy.Publisher("/camera_calibation/image_corrected",Image, queue_size=10)
      self.image_sub = rospy.Subscriber("/cam/camera_/image_raw",Image,self.cvt_image)

      """ Variables """
      self.bridge = CvBridge()
      self.latestImage = None
      self.outputImage = None
      self.process = False
      self.calibrated = False
      self.correctedImage = None
      self.mtx = None
      self.dist = None

    def cvt_image(self,data):  
      try:
        self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")	
      except CvBridgeError as e:
        print(e)
      if self.process != True:
          self.process = True    
      
    def camera_cal(self, image):
        
        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        nx = 8
        ny = 6
        
        dst = np.copy(image) 
        
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((ny * nx, 3), np.float32)
        objp[:,:2] = np.mgrid[0:nx, 0:ny].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.
        
        # Search for chessboard corners
        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        #ret_thresh,  mask = cv2.threshold(grey, 30, 255, cv2.THRESH_BINARY)
        
        ret, corners = cv2.findChessboardCorners(image, (nx, ny), None)  #flags=(cv2.cv.CV_CALIB_CB_ADAPTIVE_THRESH + cv2.cv.CV_CALIB_CB_FILTER_QUADS))        
        
        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)           
            cv2.cornerSubPix(grey,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            self.calibrated = True
            print ("FOUND!")
            
            #Draw and display the corners
            cv2.drawChessboardCorners(image, (nx, ny), corners, ret)  
            
            # Do camera calibration given object points and image points
            ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1], None, None)        
        
            # Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
            dist_pickle = {}
            dist_pickle["mtx"] = self.mtx
            dist_pickle["dist"] = self.dist
            dist_pickle['objpoints'] = objpoints
            dist_pickle['imgpoints'] = imgpoints
            pickle.dump( dist_pickle, open( "/home/wil/ros/catkin_ws/src/av_sim/computer_vision/camera_calibration/data/camera_cal_pickle.p", "wb" ) )

         #else:
             #print("Searching...")
             
        return image

    def drawQuad(self, image, points, color=[255, 0, 0], thickness=4):
        p1, p2, p3, p4 = points
        cv2.line(image, tuple(p1), tuple(p2), color, thickness)
        cv2.line(image, tuple(p2), tuple(p3), color, thickness)
        cv2.line(image, tuple(p3), tuple(p4), color, thickness)
        cv2.line(image, tuple(p4), tuple(p1), color, thickness)
    
    def perspective_transform(self,  image, debug=True, size_top=70, size_bottom=370):
        height, width = image.shape[0:2]
        output_size = height/2

        #src = np.float32([[(width/2) - size_top, height*0.65], [(width/2) + size_top, height*0.65], [(width/2) + size_bottom, height-50], [(width/2) - size_bottom, height-50]])
        src = np.float32([[512, 450], [675, 454], [707, 560], [347, 568]])
        dst = np.float32([[347, height], [707, height], [707, 0], [347, 0]])
        #dst = np.float32([[(width/2) - output_size, (height/2) - output_size], [(width/2) + output_size, (height/2) - output_size], [(width/2) + output_size, (height/2) + output_size], [(width/2) - output_size, (height/2) + output_size]])
        
        M = cv2.getPerspectiveTransform(src, dst)
        print(M)
        warped = cv2.warpPerspective(image, M, (width, height), flags=cv2.INTER_LINEAR)
        
        if debug:
            self.drawQuad(image, src, [255, 0, 0])
            self.drawQuad(image, dst, [255, 255, 0])
            plt.imshow(image)
            plt.show()
            
        return warped


    def undistort_image(self, image):
      
        return cv2.undistort(image, self.mtx, self.dist, None, self.mtx)

    def run(self):
                
         while True:
             
             # Only run loop if we have an image
             if self.process:                 
                 
                 filename = "/home/wil/ros/catkin_ws/src/av_sim/computer_vision/camera_calibration/data/check_test.png"	
                 image = cv2.imread(filename, flags=cv2.IMREAD_COLOR)
                 
                 if self.calibrated is not True:
                     #print("Calibrating...")
                     cornersImage = self.camera_cal(image)
                     cvImage = cornersImage
                     
                 else:
                     correctedImage = self.undistort_image(self.latestImage)	# Distortion Correction Function
                     transformImage = self.perspective_transform(self.latestImage)
                     cvImage = transformImage
                     
                 # Publish Undistorted Image            
                 try:
                     imgmsg = self.bridge.cv2_to_imgmsg(cvImage, "bgr8") #"mono8" "bgr8"
                     self.image_pub.publish(imgmsg)
                 except CvBridgeError as e:
                     print(e)


def main(args):

  rospy.init_node('camera_calibarion', anonymous=True)

  cc = camera_calibarion() 

  cc.run() 

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
