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
import rosbag
import os
import matplotlib.pyplot as plt

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

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

# %% Data acquisition
def ROI(image, perc):
    height, width = image.shape
    mask = np.zeros_like(image)
    
    polygon = np.array([[(0,height), (0, perc*height/100), (width, perc*height/100), (width, height)]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    crop = cv2.bitwise_and(image, mask)
    return crop

def Points_of_lines(count, avg_param, image_shape, perc_ROI):
    
    if count > 0:
        avg_param = [avg_param[0]/count, avg_param[1]/count]  # mean
    
        # Obtain 2 points for the lines
        if avg_param[0] != 0.0:   
            pt1 = (int((image_shape[0]*perc_ROI/100- avg_param[1])/avg_param[0]) , int(image_shape[0]*perc_ROI/100) )
            pt2 = (int((image_shape[0]- avg_param[1])/avg_param[0]) , image_shape[0])
            
            pt3 = (int((image_shape[0]/2 - avg_param[1])/avg_param[0]) , int(image_shape[0]/2))
        
        elif abs(avg_param[0]) == math.inf:   # handle vertical lines
            pt1 = (int(avg_param[1]), int(image_shape[0]*perc_ROI/100))
            pt2 = (int(avg_param[1]), image_shape[0])
            
            pt3 = ((int(avg_param[1]), int(image_shape[0]/2)))
        
        else:   # handle orizontal lines
            pt1 = (0 , int(avg_param[1]))
            pt2 = (image_shape[1] , int(avg_param[1]))
            
            pt3 = (0,0)
    
        return pt1, pt2, pt3
         
    return (0,0),(0,0)

def Average_lines(linesP, image, perc_ROI):

    avg_pos = [0,0] # average of slopes and coefficient for the positive side
    count_pos = 0
    avg_neg = [0,0] # average of slopes and coefficient for the negative side
    count_neg = 0
    
    if linesP is not None:
     for i in range(0, len(linesP)):           
         l = linesP[i][0]
         
         m = (l[3]-l[1])/(l[2]-l[0]) # slope
         a = l[1]-m*l[0] # coefficient
         
         if m > 0.0: # positive side
             count_pos += 1
             avg_pos = [avg_pos[0] + m, avg_pos[1] + a] #Iterative sum of slopes and coefficients for the positive side
             
             if abs(avg_pos[0]) == math.inf:    # handle vertical lines
                 avg_pos[1] = l[0]
            
         if m < 0.0: # negative side
             count_neg += 1
             avg_neg = [avg_neg[0] + m, avg_neg[1] + a] #Iterative sum of slopes and coefficients for the negative side
             
             if abs(avg_neg[0]) == math.inf:    # handle vertical lines
                 avg_neg[1] = l[0]
    
    out_lines = np.empty((0,2,2))
    distance = np.array([[0,0], [0,0], [0,0]])
    
    if count_pos > 0:
        pt1_pos, pt2_pos, pt3_pos = Points_of_lines(count_pos, avg_pos, image.shape, perc_ROI)
        out_lines = np.append(out_lines, [[pt1_pos, pt2_pos]], axis=0)
        cv2.line(image, pt1_pos, pt2_pos, (0,255,0), 3, cv2.LINE_AA)
    
    if count_neg > 0:        
        pt1_neg, pt2_neg, pt3_neg = Points_of_lines(count_neg, avg_neg, image.shape, perc_ROI)
        out_lines = np.append(out_lines, [[pt1_neg, pt2_neg]], axis=0)
        cv2.line(image, pt1_neg, pt2_neg, (0,255,0), 3, cv2.LINE_AA)
        
    if count_pos > 0 and count_neg > 0:
        distance = np.array([(pt1_pos[0]-pt1_neg[0], pt1_pos[1]), (pt2_pos[0]-pt2_neg[0], pt2_pos[1]), (pt3_pos[0]-pt3_neg[0], pt3_pos[1])])
    
    return out_lines, distance
    
def Mid_line(lines, image):
    
    height, width = image.shape
    
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
    
    angle_mid_deg = int(math.degrees(math.atan((pt2[0]-pt1[0])/(pt2[1]-pt1[1]))))
    cv2.putText(image, str(angle_mid_deg) ,(10,300), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 0), 2, cv2.LINE_AA)
    
    return angle_mid_deg 
         
t0 = time.time()
bag = rosbag.Bag("bags/bag1.bag", "r")
bridge = CvBridge()         
count = 0
perc_uppercut_ROI = 33
#fig = plt.figure()
#plt.show()
distances = np.empty((0,3,2))

for topic, msg, t in bag.read_messages(topics=["/raspicam_node/image/compressed"]):
    cv_img = bridge.compressed_imgmsg_to_cv2( msg, desired_encoding="passthrough")
    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    #cv_img = cv2.inRange(cv_img, np.array([0,0,0]), np.array([50,40,40]))
    
    cv_img = cv2.inRange(cv_img,0,25)
    
    image_unf =  cv2.undistort(cv_img, camera_matrix, dist_coeff) 
    image_unf_C = cv2.Canny(image_unf, 40, 90, 3)
    
    linesP = cv2.HoughLinesP(ROI(image_unf_C, perc_uppercut_ROI), 1, np.pi / 180, 100, None, 50, 40)
    
    #image_unf = cv2.cvtColor(image_unf, cv2.COLOR_GRAY2BGR)
         
    lines, distance = Average_lines(linesP, image_unf, perc_uppercut_ROI)
    
    distances = np.append(distances, [distance], axis = 0)
    
    if len(lines)>0:
        angle = Mid_line(lines, image_unf)

    #plt.clf()
    #fig = plt.scatter(ln_eq[:,0], ln_eq[:,1])
    #plt.pause(0.0001)

    cv2.imshow('Lines', image_unf)
    cv2.imshow('Edges', image_unf_C)
    
    k = cv2.waitKey(0)
    if k == 27:
        cv2.destroyAllWindows()
        break              
cv2.destroyAllWindows()
bag.close()

# %%
#image = cv2.imread('snapshot1.jpg', 0)

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
          
        
        
            
    
    
        



