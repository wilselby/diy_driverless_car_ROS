#!/usr/bin/env python2.7

# Python libraries
import sys, serial, struct, time

# Import OpenCV
import cv2
import numpy as np

# Import ROS
import rospy
import roslib
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class webcam_client(object):
    def __init__(self):
        self.cam = None
        self.img = None
        self.key = 0
        self.bridge = CvBridge()
        self.pub_raw = rospy.Publisher("/openmv_cam/image/raw", Image, queue_size = 10)
        #self.pub_comp = rospy.Publisher("/image_compressed", CompressedImage, queue_size = 10)


    def initialize(self):

        self.cam = cv2.VideoCapture(0)

    def read_image(self):

        ret_val, self.img = self.cam.read()

    def publish_image(self):

        """
        compressed = CompressedImage()
        compressed.header.stamp = rospy.Time.now()
        compressed.format = "jpeg"
        compressed.data = np.array(cv2.imencode('.jpg', self.img)[1]).tostring()
        self.pub_comp.publish(compressed)
        """

        try:
            if np.size(self.img.shape) == 3:
                imgmsg = self.bridge.cv2_to_imgmsg(self.img, 'bgr8')
            else:
                imgmsg = self.bridge.cv2_to_imgmsg(self.img, 'mono8')
            imgmsg.header.stamp = rospy.Time.now() #Adjust for 10hz rate? -.1?
            imgmsg.header.frame_id = "openmv_cam"
            self.pub_raw.publish(imgmsg)
        except CvBridgeError as e:
            print(e)


    def show_image(self):
        cv2.imshow("Stream:", self.img)
        self.key = cv2.waitKey(20)

    def run(self):

        self.initialize()

        #pub = initialize_publishers()

        while not rospy.is_shutdown():

            rate = rospy.Rate(30) # Run at 10Hz

            #start_time = time.time()
            self.read_image()
            #self.show_image()
            self.publish_image()            
            
            if self.key == 27:
                #seial_port.close()
                cv2.destroyWindow("preview")          
                break

            rate.sleep()
            #print("FPS: ", 1.0/(time.time()-start_time))

def main(args):

    rospy.init_node('webcam', anonymous=True)

    cam = webcam_client()

    cam.run()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# Main
if __name__ == '__main__':
    main(sys.argv)


    
         
