/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/**
 * Class implementing a basic Joy -> Twist translation.
 */
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

	std::map <int, std::string> xb360_buttons_map = {{0, "A"}, {1, "B"}, {2, "X"}, {3, "Y"}, {4, "LB"}, {5, "RB"}};
	std::map <int, std::string> xb360_axes_map = {{0, "Left stick orizzontal direction"}, {1, "Left stick vertical direction"}, {2, "LT"}, {3, "Right stick orizzontal direction"}, {4, "Right stick vertical direction"}, {5, "RT"}};

  	int enable_button = 0;
	int forward_axis = 5;
	int backward_axis = 2;
	int angular_axis = 0;

	double out_scale = 0.5;

	void joyCallback(const sensor_msgs::Joy& msg);
	double linear_motion(const double forward, const double backward);


};

} 

//#endif  // XB360_TELEOP_INTERFACE_XB360_TELEOP_INTERFACE_H
