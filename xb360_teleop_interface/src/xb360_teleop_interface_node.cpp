/**

\authors   		Ferraro Stefano <ferrarostefano@rocketmail.com>

\date			18/10/2020

\description	The package provide a functional interface for the control of the Legocar.

*/

#include "ros/ros.h"
#include "xb360_teleop_interface/xb360_teleop_interface.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "xb360_teleop_interface_node");

  ros::NodeHandle nh("");

  xb360_teleop_interface::Interface joy_teleop(&nh);

  ros::spin();
}
