#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Fri Oct  9 17:16:38 2020

@author: stefano ferraro

Program for acquiring an image and detencting the lines. A mid line for the frame is computed.
The angle of the latter is then used for controlling the steering and speed (through a PD controller)
of the LegoCar.

"""
import sys
import cv2
import time
import yaml

import rospy
import numpy as np
import math

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from line_detect.msg import steering_angle

class LineDetector:
    
    def __init__(self):
        
        print("Wainting for camera informations")
        camera_info = rospy.wait_for_message("raspicam_node/camera_info", CameraInfo)
        
        #calibration_file = open("/home/ubuntu/.ros/camera_info/camerav2_640x480.yaml")
        #calibration_data = yaml.load(calibration_file)
        self.camera_matrix = np.array(camera_info.K).reshape(3,3)
        self.dist_coeff = camera_info.D
        
        self.pub1 = rospy.Publisher('/line_detect/image/compressed', CompressedImage, queue_size=1)
        self.pub2 = rospy.Publisher('/steering_angle', steering_angle , queue_size=1)
        self.pub3 = rospy.Publisher('/line_detect/imageC/compressed', CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.callback, queue_size = 1)


    def callback(self, Img_data):
        
        perc_uppercut_ROI = 33
        np_arr = np.fromstring(Img_data.data, np.uint8)
        image_g = cv2.cvtColor(cv2.imdecode(np_arr, 1), cv2.COLOR_BGR2GRAY)
        image_un = cv2.undistort(image_g, self.camera_matrix, self.dist_coeff)
        
        image_g_filtered = cv2.inRange(image_un,0,50)

        image_C = cv2.Canny(image_g_filtered, 40, 90)
        
        linesP = cv2.HoughLinesP(self.ROI(image_C, perc_uppercut_ROI), 1, np.pi / 180, 100, None, 50, 40)
        
        image_un = cv2.cvtColor(image_un, cv2.COLOR_GRAY2BGR)
            
        if linesP is not None:
            for i in range(0, len(linesP)):
                 l = linesP[i][0]
                 cv2.line(image_un, (l[0], l[1]), (l[2], l[3]), (0,100,0), 3, cv2.LINE_AA)
         
        lines = self.Average_lines(linesP, image_un, perc_uppercut_ROI)
        
        if len(lines)>0:
            angle = self.Mid_line(lines, image_un)
            msg2 = steering_angle()
            msg2.ang = angle
            self.pub2.publish(msg2)
        
        msg1 = CompressedImage()
        msg1.format = "jpeg"
        msg1.data = np.array(cv2.imencode('.jpg', image_un)[1]).tostring()
        self.pub1.publish(msg1)
        
        msg3 = CompressedImage()
        msg3.format = "jpeg"
        msg3.data = np.array(cv2.imencode('.jpg', image_C)[1]).tostring()
        self.pub3.publish(msg3)

        
    def ROI(self, image, perc):
        
        height, width = image.shape
            
        mask = np.zeros_like(image)
        
        polygon = np.array([[(0,height), (0, perc*height/100), (width, perc*height/100), (width, height)]], np.int32)
        
        cv2.fillPoly(mask, polygon, 255)
        crop = cv2.bitwise_and(image, mask)
        return crop
    
    def Points_of_lines(self, count, avg_param, image_shape, perc_ROI):
    
        if count > 0:
            avg_param = [avg_param[0]/count, avg_param[1]/count]  # mean
        
            # Obtain 2 points for the lines
            if avg_param[0] != 0.0:   
                pt1 = (int((image_shape[0]*perc_ROI/100- avg_param[1])/avg_param[0]) , int(image_shape[0]*perc_ROI/100) )
                pt2 = (int((image_shape[0]- avg_param[1])/avg_param[0]) , image_shape[0])
            
            elif abs(avg_param[0]) == math.inf:   # handle vertical lines
                pt1 = (int(avg_param[1]), int(image_shape[0]*perc_ROI/100))
                pt2 = (int(avg_param[1]), image_shape[0])
            
            else:   # handle orizontal lines
                pt1 = (0 , int(avg_param[1]))
                pt2 = (image_shape[1] , int(avg_param[1]))
        
            return pt1, pt2
        
        return (0,0),(0,0)
    
    def Average_lines(self, linesP, image, perc_ROI):

        avg_pos = [0.0,0.0] # average of slopes and coefficient for the positive side
        count_pos = 0
        avg_neg = [0.0,0.0] # average of slopes and coefficient for the negative side
        count_neg = 0
        
        if linesP is not None:
         for i in range(0, len(linesP)):        
             
             l = linesP[i][0]
             
             if float(l[2]-l[0]) > 0:
                 m = (float(l[3]-l[1])/float(l[2]-l[0])) # slope
                 a = l[1]-float(m*l[0]) # coefficient
            
             else:  # handle vertical lines (slope to inf)
                 m = float('inf')
                 a = float('inf')
             
             if m > 0.0: # positive side
                 count_pos = count_pos + 1
                 avg_pos = [avg_pos[0] + m, avg_pos[1] + a] #Iterative sum of slopes and coefficients for the positive side
                 
                 if abs(avg_pos[0]) == float('inf'):    # handle vertical lines
                     avg_pos[1] = l[0]
                
             if m < 0.0: # negative side
                 count_neg = count_neg + 1
                 avg_neg = [avg_neg[0] + m, avg_neg[1] + a] #Iterative sum of slopes and coefficients for the negative side
                 
                 if abs(avg_neg[0]) == float('inf'):    # handle vertical lines
                     avg_neg[1] = l[0]
        
        out_lines = np.empty((0,2,2))
        
        if count_pos > 0:
            pt1_pos, pt2_pos = self.Points_of_lines(count_pos, avg_pos, image.shape, perc_ROI)
            out_lines = np.append(out_lines, [[pt1_pos, pt2_pos]], axis=0)    
            cv2.line(image, pt1_pos, pt2_pos, (0,255,0), 3, cv2.LINE_AA)
        if count_neg > 0:        
            pt1_neg, pt2_neg = self.Points_of_lines(count_neg, avg_neg, image.shape, perc_ROI)
            out_lines = np.append(out_lines, [[pt1_neg, pt2_neg]], axis=0)
            cv2.line(image, pt1_neg, pt2_neg, (0,255,0), 3, cv2.LINE_AA)
        
        return out_lines
        
    def Mid_line(self, lines, image):
    
        height, width, _ = image.shape
    
        if len(lines) == 2:
            pt1 = (int((lines[0][0][0]+lines[1][0][0])/2), int(lines[0][0][1]))  
            pt2 = (int(width/2), int(height))
        if len(lines) == 1:
            if lines[0][0][0] > lines[0][1][0]:
                pt1 = (int((lines[0][0][0]) + 380/2), int(lines[0][0][1]))  
            else:
                pt1 = (int((lines[0][0][0]) - 380/2), int(lines[0][0][1]))  
            pt2 = (int(width/2), height)
            
        cv2.line(image, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
        rad = float(pt2[0]-pt1[0])/(pt2[1]-pt1[1])
        angle_mid_deg = int(math.degrees(math.atan(rad)))
        
        return angle_mid_deg 
         
        
def main(args):
    rospy.init_node('line_detect_node')
    ld = LineDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Line Detector module")
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)

        
        
            
    
    
        



