# **Line Detection**

 <img src="../pics/gif/midline.gif" alt="midgif" width = 300>
 
The package provide lane detection functionalities for the compressed image coming from the `raspicam_node`. A direction angle is computed and outputted with the topic `/steering_angle`.
The Implementation presented is based on several filters applied to the image, and the subsequent application of Canny algorithm + Hought transform. 

## **1 - Main dependencies**
 
* sensor_msgs/CameraInfo
* sensor_msgs/CompressedImage
* rospy
* numpy
* opencv (cv2)
* line_detect/steering_angle

## **2 - Functionalities**

The package subscribe to the topic `/raspicam_node/image/compressed` (type sensor_msgs/CompressedImage), published by the raspicam_node package. Messages are published on topic `/steering_angle`, a custom message designed for passing direction information to the PD controller.

The acquired image go through a specific pipeline in order to extract the the lane/s from the scene. Lanes are then implied in the computation of a direction vector for the LegoCar.

The pipeline of the image analysis process is:

 1. **Image acquisition and undistortion process**
 2. **Filtering image (blacks)**
 3. **Edge detection wt Canny algorithm**
 4. **ROI + Lines detection wt Hough transform**
 5. **Averaging of detected lines based on slope**
 6. **Computation of middle line**
 
 <img src="../pics/lane_detection_pipeline.png" alt="screen">

### **2.1 - Image acquisition and undistortion process** 
 
Cameras transform 3D space information into a 2D form. Since this process is not perfect (due to the imperfect production process of the lenses), we cannot trust the information coming straight from the camera as it is. Our goal is to position the car in space, in order to do it properly the camera matrix and the distortion coefficient needs to be computed and both compensated.

 <img src="../pics/grey_undistort.png" alt="screen" width = 800 >







