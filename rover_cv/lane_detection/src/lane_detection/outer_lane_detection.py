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
from geometry_msgs.msg import Twist,  TwistStamped

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
      self.blurImage = None
      self.edgeImage = None
      self.maskedImage = None
      self.lineMarkedImage = None
      self.imgRcvd = False
      
      self.kernel_size = 11
      self.low_threshold = 40
      self.high_threshold = 50
      self.rho = 1
      self.theta = np.pi/180
      self.threshold = 100
      self.min_line_len = 60
      self.max_line_gap = 80
      self.lines = (0, 0, 0, 0)
      
      self.intersectionPoint = (0,  0)  
      self.speed = 0.2
      self.flag = 0

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
             
             # step 2: perspective transform
             
             # step 3: detect binary lane markings
             self.blurImage = ld.gaussian_blur(self.latestImage, self.kernel_size)             
             self.edgeImage = ld.canny(self.blurImage, self.low_threshold, self.high_threshold)
             
             #Define region of interest for cropping
             height = self.latestImage.shape[0]
             width = self.latestImage.shape[1]
             
             vertices = np.array( [[
                    [4*width/4, 3*height/5],
                    [0*width/4, 3*height/5],
                    [10, height],
                    [width-10, height]
                ]], dtype=np.int32 )
            
             self.maskedImage = ld.region_of_interest(self.edgeImage, vertices)
             
             self.lines = ld.hough_lines(self.maskedImage, self.rho, self.theta, self.threshold, self.min_line_len, self.max_line_gap)
             
             self.lineMarkedImage, self.intersectionPoint  = ld.draw_lane_lines(self.latestImage, self.lines)

             self.flag = control.adjustMotorSpeed(self.latestImage,  self.intersectionPoint,  self.speed,  self.cmdVelocityPub, self.cmdVelocityStampedPub, self.flag)

             # Publish Processed Image
             self.outputImage = self.lineMarkedImage
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
