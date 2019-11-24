#!/usr/bin/env python2.7

# Python libraries
import sys, serial, struct, time, os

# Import OpenCV
import cv2
import numpy as np
import time


port = '/dev/video0'

# Check for calibration data
cal_path = './calibration/output/calib.npz'
calibration = 0
if os.path.isfile(cal_path):
    cal = np.load(cal_path)
    K = cal['camera_matrix']
    D = cal['dist_coefs']
    calibration = 1

cam = cv2.VideoCapture(0)

start = time.time()
interval = .5
counter = 0

while True:
    
    
    ret_val, img = cam.read()

    # Undistort the image if calibration exists
    if calibration:
        DIM = img.shape[1::-1]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    counter += 1

    if (time.time() - start) > interval:
        fps = counter / (time.time() - start)
        print("FPS: {} \n").format(fps)
        counter = 0
        start = time.time()

    cv2.imshow("Stream:", img)
    key = cv2.waitKey(20)
    #print(key)

    if key == 27:
        #seial_port.close()
        cv2.destroyWindow("Stream:")  
	break      

         
