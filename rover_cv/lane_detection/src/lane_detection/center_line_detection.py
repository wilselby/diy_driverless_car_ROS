#!/usr/bin/env python
# -*- coding: utf-8 -*-
# https://github.com/paramaggarwal/CarND-LaneLines-P1/blob/master/P1.ipynb
from __future__ import print_function
from __future__ import division
import roslib
roslib.load_manifest('formulapi_sitl')
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
import lane_detection_module as ld
import control_module as control
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, TwistStamped

class lane_detection(object):
    def __init__(self):
            
      """ROS Subscriptions """
      self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)
      self.image_sub = rospy.Subscriber("/raspicam_node/image/image_raw",Image,self.cvt_image) 
      self.cmdVelocityPub = rospy.Publisher('/platform_control/cmd_vel', Twist, queue_size=10)
      self.cmdVelocityStampedPub = rospy.Publisher('/platform_control/cmd_vel_stamped', TwistStamped, queue_size=10)

      """ Variables """
      self.bridge = CvBridge()
      self.latestImage = None
      self.outputImage = None 
      self.maskedImage = None
      self.binaryImage = None
      self.channelImage = None
      self.maskedRGBImage = None
      self.processedImage = None
      self.imgRcvd = False
      
      # Gazebo Variables
      self.corners = np.float32([[44,560], [378,450],[902,450],[1215,560]]) #Gazebo Conde track
      self.boundaries = [([0, 0, 24], [255, 255, 255])] #Gazebo Conde track
      
      # Raspicam Variables
      #self.corners = np.float32([[15,238], [138,187],[217,187],[238,238]]) #Checkerboard
      #self.corners = np.float32([[15,238], [101,140],[189,140],[297,238]]) #Kitchen
      #self.boundaries = [([13, 102, 13], [53, 244, 34])] #LocalMotors track (HSV - yellow)
      #self.boundaries = [([28, 0, 0], [110, 150, 45])] #Kitchen (HSV - black) 
      
      self.intersectionPoint = (0,  0)  
      self.speed = 0.15
      self.flag = 0
      self.avg = 0

    def cvt_image(self,data):  
      try:
        self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")	
      except CvBridgeError as e:
        print(e)
      if self.imgRcvd != True:
          self.imgRcvd = True    
          
    def publish(self, image,  bridge,  publisher):
        try:
            #Determine Encoding
            if np.size(image.shape) == 3: 
                imgmsg = bridge.cv2_to_imgmsg(image, "bgr8") 
            else:
                imgmsg = bridge.cv2_to_imgmsg(image, "mono8") 
            publisher.publish(imgmsg)  
        except CvBridgeError as e:
            print(e)

 
    def run(self):
     
     while True:
         # Only run loop if we have an image
         if self.imgRcvd:             

             # step 1: undistort image
             
             #Define region of interest for cropping
             height = self.latestImage.shape[0]
             width = self.latestImage.shape[1]
             
             """ Gazebo Conde """
             self.vertices = np.array( [[
                        [2.75*width/5, 3*height/5],
                        [2.25*width/5, 3*height/5],
                        [.5*width/5, height], 
                        [4.5*width/5, height]
                    ]], dtype=np.int32 )
             """
             
             # Raspicam 
             original
             self.vertices = np.array( [[
                        [2.75*width/5, 3*height/5],
                        [2.25*width/5, 3*height/5],
                        [.5*width/5, height], 
                        [4.5*width/5, height]
                    ]], dtype=np.int32 )
             
             self.vertices = np.array( [[
                        [3.75*width/5, 2*height/5],
                        [1.25*width/5, 2*height/5],
                        [.05*width/5, height], 
                        [4.95*width/5, height]
                    ]], dtype=np.int32 )
             """
            
             self.maskedImage = ld.region_of_interest(self.latestImage, self.vertices)
             
             # step 2: perspective transform
             self.warpedImage,  _,  _ = ld.perspective_transform(self.maskedImage, self.corners)
             
             # step 3: detect binary lane markings
             #self.binaryImage,  self.channelImage = ld.HLS_sobel(self.warpedImage)
             self.binaryImage = ld.binary_thresh(self.warpedImage,  self.boundaries,  'HSV')     #RGB or HSV
             
             # step 4: fit polynomials
             if self.global_fit is not None:
                 ploty, fitx, fit = ld.fast_fit_polynomials(self.binaryImage,  self.global_fit)
             else:
                 ploty, fitx, fit = ld.fit_polynomials(self.warpedImage, self.binaryImage)
             
             self.global_fit = fit
             
             if self.binaryImage is not None:

                 data = cv2.cvtColor(self.binaryImage, cv2.COLOR_GRAY2BGR)
                 
                 r1, g1, b1 = 255, 255, 255 # Original value
                 r2, g2, b2 = 255, 0, 0 # Value that we want to replace it with

                 red, green, blue = data[:,:,0], data[:,:,1], data[:,:,2]
                 mask = (red == r1) & (green == g1) & (blue == b1)
                 data[:,:,:3][mask] = [b2, g2, r2]
                 
                 output = cv2.bitwise_and(self.warpedImage, self.warpedImage,  mask = self.binaryImage)   #Returns an RGB image                 

                 _,  src,  dst = ld.perspective_transform(self.latestImage, self.corners)
                 Minv = cv2.getPerspectiveTransform(dst, src)
                 
                 newwarp = cv2.warpPerspective(data, Minv, (self.latestImage.shape[1], self.latestImage.shape[0]))
                 
                 self.maskedRGBImage = cv2.addWeighted(newwarp, 1, self.latestImage, 2.0, 0)                 
             
             if fitx.shape[0] >1:
                 
                 # step 5: draw lane             
                 self.processedImage = ld.render_lane(self.maskedRGBImage, self.corners, ploty, fitx) 

                 # step 6: print curvature
                 #self.curv = get_curvature(ploty, fitx)     

                 # step 6: Calculate Setpoint
                 pts = np.vstack((fitx,ploty)).astype(np.float64).T
                 #self.avg = ld.movingAverage(self.avg, pts[-1][0],  N=20) # Elegoo Kitchen
                 self.avg = ld.movingAverage(self.avg, pts[-1][0],  N=1) # Gazebo
                 self.intersectionPoint = np.array([self.avg])
                 
                 # Draw the Setpoint onto the warped blank image
                 radius = 5
                 cv2.circle(self.processedImage,(self.intersectionPoint, (self.processedImage.shape[0]-radius)), radius, (255,255,0), -1)
                 
                 # step 6: Adjust Motors
                 self.flag = control.adjustMotorSpeed(self.latestImage,  self.intersectionPoint,  self.speed,  self.cmdVelocityPub, self.cmdVelocityStampedPub, self.flag)
             else:
                 self.processedImage = self.latestImage
                 
             # Publish Processed Image
             self.outputImage = self.processedImage
             self.publish(self.outputImage, self.bridge,  self.image_pub)


def main(args):

  rospy.init_node('center_line_detection', anonymous=True)

  ld = lane_detection() 

  ld.run() 

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
