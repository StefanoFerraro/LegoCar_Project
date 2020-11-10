/**

\authors   		Ferraro Stefano <ferrarostefano@rocketmail.com>

\date			18/10/2020

\description	Header file

*/

//#ifndef XB360_TELEOP_INTERFACE_XB360_TELEOP_INTERFACE_H
//#define XB360_TELEOP_INTERFACE_XB360_TELEOP_INTERFACE_H

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

#include <iostream>
#include <map>
#include <string>


namespace xb360_teleop_interface
{

class Interface
{
public:
	Interface(ros::NodeHandle* nh);

  	virtual~Interface();

private:
	
	geometry_msgs::Twist cmd_vel_msg;
	sensor_msgs::Joy Joy;
	ros::Subscriber joy_sub;
	ros::Publisher cmd_vel_pub;

	// Mapping of the XBOX360 buttons ans axes to integer values
	std::map <int, std::string> xb360_buttons_map = {{0, "A"}, {1, "B"}, {2, "X"}, {3, "Y"}, {4, "LB"}, {5, "RB"}};
	std::map <int, std::string> xb360_axes_map = {{0, "Left stick orizzontal direction"}, {1, "Left stick vertical direction"}, {2, "LT"}, {3, "Right stick orizzontal direction"}, {4, "Right stick vertical direction"}, {5, "RT"}};

	// Function linking to buttons/axes 
  	int enable_button = 0;
	int forward_axis = 5;
	int backward_axis = 2;
	int angular_axis = 0;

	// trigger for handling different enable buttons
	bool trig = false;

	// Output scale for throttling/steering commands
	double linear_out_scale;
	double angular_out_scale;

	// functions declaration
	void joyCallback(const sensor_msgs::Joy& msg);
	double linear_motion(const double forward, const double backward);
	double angular_motion(const double angle);


};

} 

//#endif  // XB360_TELEOP_INTERFACE_XB360_TELEOP_INTERFACE_H
