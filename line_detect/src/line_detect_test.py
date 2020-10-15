#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Fri Oct  9 17:16:38 2020

@author: stefano ferraro

Program for taking an image and detencting the line in it, having this cycle inside a while loop 
should let us analyse a video stream 

"""
import sys
import cv2
import time
import yaml
import math
# import rospy
import numpy as np
# import matplotlib.pyplot as plt

# from sensor_msgs.msg import CompressedImage
# from cv_brindge import CvBridge, CvBridgeError

# class LineDetector:
    
#     def __init__(self):
        
#         self.pub = rospy.Publisher('/line_detect/image/compressed', CompressedImage, queue_size=1)
#         self.sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.ld_callback)


#     def ld_callback(self, Img_data):
        
#         np_arr = np.fromstring(Img_data.data, np.uint8)
#         image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         image_cv_g = cv2.flip(cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY), 0)
        
        
#         cv2.imshow('cv_img', image_cv_g)
#         msg = CompressedImage()
#         msg.format = "jpeg"
#         msg.data = np.array(cv2.imencode('.jpg', image_cv_g)[1]).tostring()
#         self.pub.publish(msg)
        
# def main(args):
#     ld = LineDetector()
#     rospy.init_node('line_detect_node')
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down ROS Line Detector module")
#     cv2.destroyAllWindows()
    
# if __name__ == '__main__':
#     main(sys.argv)

calibration_file = open("camerav2_640x480.yaml")
calibration_data = yaml.load(calibration_file, Loader=yaml.FullLoader)
camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape(3,3)
dist_coeff = np.array(calibration_data['distortion_coefficients']['data'])

t0 = time.time()
image = cv2.imread('snapshot1.jpg', 0)

#cv2.imshow('cv_img', image)

t1 = time.time()

print("seconds for loading image: ", t1-t0)

image_unf =  cv2.flip(cv2.undistort(image, camera_matrix, dist_coeff),0)

#cv2.imshow('cv_img_un', image_un)

t2 = time.time()

print("seconds for undistort: ", t2-t1)

#image_unf = cv2.flip(image_un, 0)  
#cv2.imshow('cv_img', image_cv_g)
#t3 = time.time()
#print("seconds for flip: %f", t3-t2)

#image_cv_g_b = cv2.GaussianBlur(image_cv_g,(3,3),0)  
# cv2.imshow('cv_img_b', image_cv_g_b)

image_unf_C = cv2.Canny(image_unf, 50, 80, None, 3) 

t3 = time.time()
print("seconds for Canny: ", t3-t2)

#cv2.imshow('cv_img_unf_C', image_unf_C)

lines = cv2.HoughLines(image_unf_C, 2, np.pi / 180, 150, None, 0, 0)
t4 = time.time()
print("seconds for Hough Transform: ", t4-t3)

linesP = cv2.HoughLinesP(image_unf_C, 1, np.pi / 180, 100, None, 50, 40)
t5 = time.time()
print("seconds for Probabilistic Hough Transform: ", t5-t4)

image_print = cv2.flip(cv2.imread('snapshot1.jpg', 1),0)

if linesP is not None:
     for i in range(0, len(linesP)):
         l = linesP[i][0]
         cv2.line(image_print, (l[0], l[1]), (l[2], l[3]), (0,100,0), 3, cv2.LINE_AA)
            
# if lines is not None:
#     for i in range(0, len(lines)):
#         rho = lines[i][0][0]
#         theta = lines[i][0][1]
#         a = math.cos(theta)
#         b = math.sin(theta)
#         x0 = a * rho
#         y0 = b * rho
#         pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
#         pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

#         cv2.line(image_print, pt1, pt2, (0,100,0), 3, cv2.LINE_AA)

#cv2.imshow('cv_img_un', image_print)
cv2.waitKey(0)
cv2.destroyAllWindows()
          
        
        
            
    
    
        



