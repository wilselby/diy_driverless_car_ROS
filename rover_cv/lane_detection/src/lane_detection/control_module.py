#!/usr/bin/env python
# -*- coding: utf-8 -*-
# https://github.com/paramaggarwal/CarND-LaneLines-P1/blob/master/P1.ipynb
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
import lane_detection_module as ld
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist,  TwistStamped

def publishCmdVel(cmdvel, velPublisher, velPublisherStamped):
    
     velPublisher.publish(cmdvel)
     
     baseVelocity = TwistStamped()

     baseVelocity.twist = cmdvel
     
     now = rospy.get_rostime()
     baseVelocity.header.stamp.secs = now.secs
     baseVelocity.header.stamp.nsecs = now.nsecs
     velPublisherStamped.publish(baseVelocity)

def adjustMotorSpeed(image,  pos,  speed,  velPublisher,  velPublisherStamped,   flag):    
     cmdvel = Twist()
     sim_time = rospy.Time()
     dt = 0
     cp = 0
     vel_er = 0
     cd = 0
     Kp = .005 #Gazebo
     Kd = .01  #Gazebo
     #Kp = .02 #Elegoo
     #Kd = .003   #Elegoo
     position_er = 0
     global position_er_last
     global last_time
     
     if flag == 0:
         last_time = rospy.Time()
         position_er_last = 0
         flag = 1
         cmdvel.linear.x = 0.0
         cmdvel.angular.z = 0.0
         publishCmdVel(cmdvel,  velPublisher,  velPublisherStamped)
         return flag
     
     if (math.isnan(pos[0])== False and math.isnan(pos[0]) == False):
        
         cmdvel.linear.x = speed

         sim_time = rospy.Time.now()
         dt = (sim_time - last_time).to_sec();             

         position_er = image.shape[1]/2 - pos[0]
         cp = position_er * Kp 
         vel_er = (position_er - position_er_last) * dt
         cd = vel_er * Kd

         cmdvel.angular.z = cp - cd
         cmdvel.angular.z = ld.limit(cmdvel.angular.z, -1.57, 1.57)
       
         publishCmdVel(cmdvel,  velPublisher,  velPublisherStamped)

         position_er_last = position_er
         last_time = sim_time
     else:
         cmdvel.linear.x = 0.0
         cmdvel.angular.z = 0.0
         publishCmdVel(cmdvel,  velPublisher,  velPublisherStamped)
         
     return flag


