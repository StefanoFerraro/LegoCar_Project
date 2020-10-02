#include <xb360_teleop_interface/xb360_teleop_interface.h>

namespace xb360_teleop_interface

{
	Interface::Interface(ros::NodeHandle* nh)
	{
		cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
		joy_sub = nh->subscribe("/joy", 1, &Interface::joyCallback, this);

		ROS_INFO_STREAM("Teleop enable button: " << xb360_buttons_map[enable_button]);
		ROS_INFO_STREAM("Teleop forward axis: " << xb360_axes_map[forward_axis]);
		ROS_INFO_STREAM("Teleop backwards axis: " << xb360_axes_map[backward_axis]);
		ROS_INFO_STREAM("Teleop angular axis: " << xb360_axes_map[angular_axis]);
		ROS_INFO_STREAM("linear axis (x) scale : " << out_scale);
		ROS_INFO_STREAM("angular axis (z) scale: " << out_scale);
	}

	Interface::~Interface()
	{

	}

	double Interface::linear_motion(const double forward, const double backward)
	{
		double frw_inscale = (-(forward - 1)/2 * out_scale);
		double bkw_inscale = ((backward - 1)/2 * out_scale);

		if(frw_inscale > 0.1 && frw_inscale > -bkw_inscale)
		{
			return -frw_inscale;	//Opposite direction of the motor
		}

		if(-bkw_inscale > 0.1 && frw_inscale < -bkw_inscale)
		{
			return -bkw_inscale;	//Opposite direction of the motor
		}
		else {return 0.0;}
	}

	void Interface::joyCallback(const sensor_msgs::Joy& msg)
	{
		if(msg.buttons[enable_button])
		{
			cmd_vel_msg.linear.x = Interface::linear_motion(msg.axes[forward_axis], msg.axes[backward_axis]);
			cmd_vel_msg.angular.z =  - msg.axes[angular_axis] * out_scale;

			cmd_vel_pub.publish(cmd_vel_msg);
		}
		else
		{
			geometry_msgs::Twist cmd_vel_zero;
			cmd_vel_pub.publish(cmd_vel_zero);			
		}
	}


}