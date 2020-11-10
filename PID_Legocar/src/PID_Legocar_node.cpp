/**

\authors   		Ferraro Stefano <ferrarostefano@rocketmail.com>

\date			1/11/2020

\description	The package provide an interface for the line detect module over the l298n module.

*/

#include "ros/ros.h"
#include "PID_Legocar/PID_Legocar.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "PID_Legocar_node");

  ros::NodeHandle nh("");

  PID_Legocar::PID PID(&nh);

  ros::spin();
}