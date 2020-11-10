/**

\authors   		Ferraro Stefano <ferrarostefano@rocketmail.com>

\date			18/10/2020

\description	The package subscribe to the message `/joy` (type sensor_msgs/Joy). 
				Sensed buttons/axes are converted into linear and angular motion.
				A message `/cmd_vel` (type geometry_msgs/Twist) is published.

*/

#include <xb360_teleop_interface/xb360_teleop_interface.h>

namespace xb360_teleop_interface

{
	Interface::Interface(ros::NodeHandle* nh)
	{
		nh->getParam("Legocar/linear_range", linear_out_scale);
		nh->getParam("Legocar/angular_range", angular_out_scale);

		cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
		joy_sub = nh->subscribe("/joy", 1, &Interface::joyCallback, this);

		// Output bottons/axes to be used for the control
		ROS_INFO_STREAM("Teleop enable button: " << xb360_buttons_map[enable_button]);
		ROS_INFO_STREAM("Teleop forward axis: " << xb360_axes_map[forward_axis]);
		ROS_INFO_STREAM("Teleop backwards axis: " << xb360_axes_map[backward_axis]);
		ROS_INFO_STREAM("Teleop angular axis: " << xb360_axes_map[angular_axis]);
		ROS_INFO_STREAM("linear axis (x) scale : " << linear_out_scale);
		ROS_INFO_STREAM("angular axis (z) scale: " << angular_out_scale);
	}

	Interface::~Interface()
	{

	}

	double Interface::linear_motion(const double forward, const double backward)
	{
		// conversion of the sensed axis into the output scale desired
		double frw_inscale = (-(forward - 1)/2 * linear_out_scale);
		double bkw_inscale = ((backward - 1)/2 * linear_out_scale);

		// distinguish between forward and backward motion, comparing the axes values
		if(frw_inscale > 0.1 && frw_inscale > -bkw_inscale)
		{
			return -frw_inscale;	// opposite direction of the motor
		}

		if(-bkw_inscale > 0.1 && frw_inscale < -bkw_inscale)
		{
			return -bkw_inscale;	// opposite direction of the motor
		}
		else {return 0.0;}
	}

	double Interface::angular_motion(const double angle)
	{
		return  -angle * angular_out_scale; // opposite direction of the servo
	}

	void Interface::joyCallback(const sensor_msgs::Joy& msg)
	{
		if(msg.buttons[enable_button]) // check if the enable button is active
		{
			trig = true;
			// linear motion conversion
			cmd_vel_msg.linear.x = Interface::linear_motion(msg.axes[forward_axis], msg.axes[backward_axis]);
			// angular motion conversion
			cmd_vel_msg.angular.z =  Interface::angular_motion(msg.axes[angular_axis]);

			cmd_vel_pub.publish(cmd_vel_msg);
		}
		else	// otherwise publish a null Twist message
		{
			if(trig)	// in order to do not have overlap of messages coming from different modes, a trigger is used to send the null message once 
			{
				geometry_msgs::Twist cmd_vel_zero;
				cmd_vel_pub.publish(cmd_vel_zero);
				trig = false;	// after executed once the trigger is disabled
			}	
		}
	}


}