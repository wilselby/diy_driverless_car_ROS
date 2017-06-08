#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image,  CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import time
import sys

class cmd_vel_node(object):
    def __init__(self):
            
      """ROS Subscriptions """
      self.cmdVelSub = rospy.Subscriber("/joy_teleop/cmd_vel", Twist, self.callback)
      self.baseVelocityPub = rospy.Publisher('/cmd_vel_stamped', TwistStamped, queue_size=10)

      """ Variables """
      self.secs = 0
      self.nsecs = 0
      self.last = 0
      self.cmdvel = Twist()
      self.baseVelocity = TwistStamped()
      self.flag = 0

        
    def callback(self, cmdVelocity):
        self.baseVelocity.twist = cmdVelocity
        self.flag = 1
        
        if self.flag:
            # Publish Updated TwistStamped
     
             baseVelocity = TwistStamped()

             baseVelocity.twist = cmdVelocity
         
             now = rospy.get_rostime()
             baseVelocity.header.stamp.secs = now.secs
             baseVelocity.header.stamp.nsecs = now.nsecs
             self.baseVelocityPub.publish(baseVelocity)
            
    
    def run(self):
        
        while True:
            self.last = 1
                  
    
def main(args):

  rospy.init_node('cmd_vel_listener', anonymous=True)

  cmd = cmd_vel_node() 

  cmd.run() 

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
    main(sys.argv)
    
