# **PID Controller Lego Car**

The package provide a functional interface between the line detection package and the l298n driver. The `/steering_angle` message coming from the line detect model is used for direct control of the steering of the LegoCar, a PD controller ('I' contribution present but not used) has been used in order to adjust throttling while steering.

## **1 - Main dependencies**

* geometry_msgs/Twist
* sensor_msgs/Joy
* ros/ros
* line_detect/steering_angle

## **2 - Functionalities**

The package subscribe to the topic `/steering_angle` (type line_detect/steering_angle), published by the [line detect](../line_detect) package. Messages are published on topic `/cmd_vel` (type geometry_msgs/Twist).

Subscription to the `/joy` topic is required for having a enable button in order to switch on/off the following controller.

