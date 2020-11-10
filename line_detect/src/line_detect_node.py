#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Fri Oct  9 17:16:38 2020

@author: stefano ferraro

Program for acquiring an image and detencting the max 2 lines, based on the slope. A mid line for the frame is computed.
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

# class for line detection of max 2 lines, based on the slope
class LineDetector:
    
    # init function for 
    def __init__(self):
            
        # fetching camera matricies form camera_info topic
        camera_info = rospy.wait_for_message("raspicam_node/camera_info", CameraInfo)
        
        self.camera_matrix = np.array(camera_info.K).reshape(3,3)
        self.dist_coeff = camera_info.D
        
        # pub/sub to required topics
        self.pub1 = rospy.Publisher('/line_detect/image/compressed', CompressedImage, queue_size=1)
        self.pub2 = rospy.Publisher('/steering_angle', steering_angle , queue_size=1)
        self.sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.callback, queue_size = 1)


    def callback(self, Img_data):
        
        perc_uppercut_ROI = 33  # define Region Of Intrest for the image (in percentage)(rectangular shape). 
                                # Top 0% (no remove) --> Bottom 100% (complete remove)
        
        # convert data from compressed form into compatible opencv form, and convert to GRAY scale
        np_arr = np.fromstring(Img_data.data, np.uint8)
        image_g = cv2.cvtColor(cv2.imdecode(np_arr, 1), cv2.COLOR_BGR2GRAY)
        
        # undistort image
        image_un = cv2.undistort(image_g, self.camera_matrix, self.dist_coeff)
        
        # filter GRAY scale image for blacks (black tape is used as lanes)
        image_g_filtered = cv2.inRange(image_un,0,50)
        
        # Canny algorithm as edge detector, coefficients used comes from experience
        image_C = cv2.Canny(image_g_filtered, 40, 90)
        
        # Hough lines (proportional) as line detector, coefficients used comes from experience 
        linesP = cv2.HoughLinesP(self.ROI(image_C, perc_uppercut_ROI), 1, np.pi / 180, 100, None, 50, 40)
        
        # convert image to BGR for plotting colored lines in gray scale image
        image_un = cv2.cvtColor(image_un, cv2.COLOR_GRAY2BGR)
        
        # loop for plotting all the lines detected
        if linesP is not None:
            for i in range(0, len(linesP)):
                 l = linesP[i][0]
                 cv2.line(image_un, (l[0], l[1]), (l[2], l[3]), (0,100,0), 3, cv2.LINE_AA)
        
        # compute average lines for positive and negative slopes
        lines = self.Average_lines(linesP, image_un, perc_uppercut_ROI)
        
        # compute and publish mid line angle for automatic steering of the vehicle
        if len(lines)>0:
            angle = self.Mid_line(lines, image_un)
            msg2 = steering_angle()
            msg2.ang = angle
            self.pub2.publish(msg2)
    
        # publish modified image        
        msg1 = CompressedImage()
        msg1.format = "jpeg"
        msg1.data = np.array(cv2.imencode('.jpg', image_un)[1]).tostring()
        self.pub1.publish(msg1)

    # function for selecting the desired Region Of Interest. 
    def ROI(self, image, perc):
        
        height, width = image.shape
        
        # create mask
        mask = np.zeros_like(image)
        
        # The region is a rectangular shape starting from top with height = perc*height/100, and width = width 
        polygon = np.array([[(0,height), (0, perc*height/100), (width, perc*height/100), (width, height)]], np.int32)
        
        # function for filling the polygon into the mask
        cv2.fillPoly(mask, polygon, 255)
        
        # bitwise operation for cropping the desired part of the image
        crop = cv2.bitwise_and(image, mask)
        return crop
    
    # function for finding two points of the line (given m and a parameters of: y = m*x + a), 
    # y coordinates are top of the ROI section and bottom of image
    def Points_of_lines(self, avg_param, image_shape, perc_ROI):
        
        # Obtain 2 points for the lines
        if avg_param[0] != 0.0:     # non vertical/horizontal lines 
            pt1 = (int((image_shape[0]*perc_ROI/100- avg_param[1])/avg_param[0]) , int(image_shape[0]*perc_ROI/100) )
            pt2 = (int((image_shape[0]- avg_param[1])/avg_param[0]) , image_shape[0])
        
        elif abs(avg_param[0]) == math.inf:   # handle vertical lines
            pt1 = (int(avg_param[1]), int(image_shape[0]*perc_ROI/100))
            pt2 = (int(avg_param[1]), image_shape[0])
        
        else:   # handle orizontal lines
            pt1 = (0 , int(avg_param[1]))
            pt2 = (image_shape[1] , int(avg_param[1]))
    
        return pt1, pt2
    
    # function for averaging all the lines outputed from the Hough transform 
    def Average_lines(self, linesP, image, perc_ROI):
        
        # y = m*x + a the following refers to the standard equation for a line, lines are grouped based on the slope (m)
        avg_pos = [0.0,0.0] # average of slopes and a coefficient for the positive side
        count_pos = 0
        avg_neg = [0.0,0.0] # average of slopes and a coefficient for the negative side
        count_neg = 0
        
        if linesP is not None:
         for i in range(0, len(linesP)):        
             
             l = linesP[i][0]
             
             if float(l[2]-l[0]) > 0:
                 m = (float(l[3]-l[1])/float(l[2]-l[0])) # slope m
                 a = l[1]-float(m*l[0]) # coefficient a
            
             else:  # handle vertical lines (slope to inf)
                 m = float('inf')
                 a = float('inf')
             
             if m > 0.0: # sum positive slope parameters
                 count_pos = count_pos + 1
                 avg_pos = [avg_pos[0] + m, avg_pos[1] + a] # Iterative sum of slopes and coefficients for the positive side
                 
                 if abs(avg_pos[0]) == float('inf'):    # handle vertical lines
                     avg_pos[1] = l[0]
                
             if m < 0.0: # sum negative slope parameters
                 count_neg = count_neg + 1
                 avg_neg = [avg_neg[0] + m, avg_neg[1] + a] #Iterative sum of slopes and coefficients for the negative side
                 
                 if abs(avg_neg[0]) == float('inf'):    # handle vertical lines
                     avg_neg[1] = l[0]
        
        # init matrix for output computed line/s
        out_lines = np.empty((0,2,2))
        
        if count_pos > 0:
            avg_pos = [avg_pos[0]/count_pos, avg_pos[1]/count_pos]  # mean of m, a parameters
            pt1_pos, pt2_pos = self.Points_of_lines(avg_pos, image.shape, perc_ROI) # compute points of average line
            out_lines = np.append(out_lines, [[pt1_pos, pt2_pos]], axis=0)    
            cv2.line(image, pt1_pos, pt2_pos, (0,255,0), 3, cv2.LINE_AA)    # add positive average line to image
        if count_neg > 0:
            avg_neg = [avg_neg[0]/count_neg, avg_neg[1]/count_neg]  # mean of m, a parameters
            pt1_neg, pt2_neg = self.Points_of_lines(avg_neg, image.shape, perc_ROI) # compute points of average line
            out_lines = np.append(out_lines, [[pt1_neg, pt2_neg]], axis=0)
            cv2.line(image, pt1_neg, pt2_neg, (0,255,0), 3, cv2.LINE_AA)    # add negative average line to image
        
        return out_lines
    
    # function for computing the mid line useful for the steering control 
    def Mid_line(self, lines, image):
    
        height, width, _ = image.shape
    
        if len(lines) == 2: # if both lines are detected, the mid line is just the mean of the two
            pt1 = (int((lines[0][0][0]+lines[1][0][0])/2), int(lines[0][0][1]))  
            pt2 = (int(width/2), int(height))
        
        if len(lines) == 1: # if only one of the line is detected the mid line is obtained adding a coefficient found 
                            # from data analysis
            if lines[0][0][0] > lines[0][1][0]: # in case of left line (based on slope)
                pt1 = (int((lines[0][0][0]) + 380/2), int(lines[0][0][1])) # 380/2 is derived form data analysis and just works for ROI = 33
            else:   # in case of right line (based on slope)
                pt1 = (int((lines[0][0][0]) - 380/2), int(lines[0][0][1])) # 380/2 is derived form data analysis and just works for ROI = 33 
            
            pt2 = (int(width/2), height)
        
        # add mid line to the output image
        cv2.line(image, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
        
        # compute angle of mid line
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

        
        
            
    
    
        



