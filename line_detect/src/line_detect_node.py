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
import rospy
import numpy as np
import math

from sensor_msgs.msg import CompressedImage
# from cv_brindge import CvBridge, CvBridgeError

class LineDetector:
    
    def __init__(self):
        
        calibration_file = open("camerav2_640x480.yaml")
        calibration_data = yaml.load(calibration_file)
        self.camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape(3,3)
        self.dist_coeff = np.array(calibration_data['distortion_coefficients']['data'])
        
        self.pub = rospy.Publisher('/line_detect/image/compressed', CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.ld_callback)


    def ld_callback(self, Img_data):
        
        np_arr = np.fromstring(Img_data.data, np.uint8)
        image_unf = cv2.flip(cv2.undistort(cv2.imdecode(np_arr, 0), self.camera_matrix, self.dist_coeff),0)
        image_unf_C = cv2.Canny(image_unf, 50, 100)
        #linesP = cv2.HoughLinesP(image_unf_C, 5, np.pi / 180, 200, None, 100, 100)
        
        # if linesP is not None:
        #     for i in range(0, len(linesP)):
        #         l = linesP[i][0]
        #         cv2.line(image_unf, (l[0], l[1]), (l[2], l[3]), (0,100,0), 3, cv2.LINE_AA)
        
        lines = cv2.HoughLines(image_unf_C, 5, np.pi / 180, 200, None, 0, 0)
        
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        
                cv2.line(image_unf, pt1, pt2, (0,100,0), 3, cv2.LINE_AA)


        #cv2.imshow('cv_img', image)
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_unf)[1]).tostring()
        self.pub.publish(msg)
        
def main(args):
    ld = LineDetector()
    rospy.init_node('line_detect_node')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Line Detector module")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)

        
        
            
    
    
        



