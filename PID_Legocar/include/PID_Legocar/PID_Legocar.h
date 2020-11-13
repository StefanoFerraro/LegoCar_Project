/**

\authors   		Ferraro Stefano <ferrarostefano@rocketmail.com>

\date			1/11/2020

\description	Header file

*/

//#ifndef PID_LEGOCAR_PID_LEGOCAR_H
//#define PID_LEGOCAR_PID_LEGOCAR_H

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <lane_detection/steering_angle.h>
#include <ros/ros.h>

#include <iostream>
#include <map>
#include <string>


namespace PID_Legocar
{

class PID
{
public:
	PID(ros::NodeHandle* nh);

  	virtual~PID();

private:
	
	geometry_msgs::Twist cmd_vel_msg;

	ros::Subscriber activation_sub;
	ros::Subscriber steering_sub;
	ros::Publisher cmd_vel_pub;

	// Mapping of the XBOX360 buttons ans axes to integer values
	std::map <int, std::string> xb360_buttons_map = {{0, "A"}, {1, "B"}, {2, "X"}, {3, "Y"}, {4, "LB"}, {5, "RB"}};

	// variables for the activation of the autopilot mode 
  	int enable_button = 1;
  	bool autopilot_act = false;
  	bool trig = false;

	// Maximum throttling velocity (assigned from yaml file)
	double def_vel;
	double delta_vel;
	
	// output scale
	double linear_out_scale;
	double angular_out_scale;
	
	// variable for storing last value of CTE comming from the line-detect package
	double CTE_past_cyc = 0;
	// variable for storing the previous time cycle, initialized with the time at the execution of the package
	ros::Time prev_time = ros::Time::now();

	// variation in time
	double dt;
	
	// coefficients
	double Cp;
	double Cd;
	double Ca;
	
	// contribution for the PID module
	double P_contribution;
	double D_contribution;
	double PD_value;

	// functions declaration
	void ActCallback(const sensor_msgs::Joy& msg);
	void Callback(const lane_detection::steering_angle& CTE);
	double PID_module(const lane_detection::steering_angle& CTE);


};

} 

//#endif  // PID_LEGOCAR_PID_LEGOCAR_H
