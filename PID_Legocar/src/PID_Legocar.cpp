/**

\authors   		Ferraro Stefano <ferrarostefano@rocketmail.com>

\date			10/11/2020

\description	The package subscribe to the message `/steering_angle`, a custom message coming from the line detect module. 
				Angular informations (/steering_angle) are used in order to pack a Twist message, in particular a PD controller 
				has been implemented for compensating velocity in turns.
				A message `/cmd_vel` (type geometry_msgs/Twist) is published.

*/

#include <PID_Legocar/PID_Legocar.h>

namespace PID_Legocar

{
	PID::PID(ros::NodeHandle* nh)
	{
		// param loading from confi file
		nh->getParam("Legocar/def_vel", def_vel);
		nh->getParam("Legocar/linear_range", linear_out_scale);
		nh->getParam("Legocar/angular_range", angular_out_scale);
		nh->getParam("Legocar/proportional_coeff", Cp);
		nh->getParam("Legocar/derivative_coeff", Cd);
		nh->getParam("Legocar/angular_coeff", Ca);
		
		// pub/sub to needed topics
		cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
		activation_sub = nh->subscribe("/joy", 1, &PID::ActCallback, this);
		steering_sub = nh->subscribe("/steering_angle", 1, &PID::Callback, this);

		// Output botton to be used for the control
		ROS_INFO_STREAM("Autopilot enable button: " << xb360_buttons_map[enable_button]);
	}

	PID::~PID()
	{

	}

	// PID module, I contribution it is not needed
	double PID::PID_module(const line_detect::steering_angle& CTE)
	{
		// offset for applying a variation in the setpoint of the velocity, to compensate the steering.
		delta_vel = linear_out_scale - def_vel;

		// computation of dt for derivative
		dt = prev_time.toSec()-ros::Time::now().toSec();

		// proportional contribution
		P_contribution = Cp*abs(CTE.ang);
		ROS_INFO_STREAM("P contribution: " << P_contribution);
		
		// integral contribution NOT NEEDED
		// I += CTE.ang*dt;
		// I_contribution = Ci*I;
		// ROS_INFO_STREAM("I contribution: " << I_contribution);

		// derivative contribution
		D_contribution = Cd*(CTE_past_cyc - abs(CTE.ang))/dt;
		ROS_INFO_STREAM("D contribution: " << D_contribution);

		// save CTE value for next iteration
		CTE_past_cyc = abs(CTE.ang);
		prev_time = ros::Time::now();

		PD_value = P_contribution + D_contribution;

		ROS_INFO_STREAM("PID_value output: " << PD_value);

		if (PD_value >= 1)	// saturate output
		{
			PD_value = 1; 
		}
		
		return PD_value * delta_vel; 
	}

	void PID::Callback(const line_detect::steering_angle& CTE)
	{
		if(autopilot_act) // check if the enable button is active
		{
			trig = true;
			// linear motion conversion
			cmd_vel_msg.linear.x = def_vel + PID::PID_module(CTE); 
			// angular motion conversion
			cmd_vel_msg.angular.z = Ca*CTE.ang;

			cmd_vel_pub.publish(cmd_vel_msg);
		}
		else	// otherwise publish a null Twist message
		{
			if (trig)	// in order to do not have overlap of messages coming from different modes, a trigger is used to send the null message once 
			{
				geometry_msgs::Twist cmd_vel_zero;
				cmd_vel_pub.publish(cmd_vel_zero);
				trig = false;	// after executed once the trigger is disabled
			}		
		}
	}


	void PID::ActCallback(const sensor_msgs::Joy& msg)
	{
		if(msg.buttons[enable_button]) // check if the enable button is active
		{
			autopilot_act = true;
		}
		else
		{
			autopilot_act = false; 		
		}
	}


}
