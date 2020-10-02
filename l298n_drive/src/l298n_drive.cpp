#include <wiringPi.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define In1		16
#define In2		15
#define En1		1
#define In3		28
#define In4		27
#define En2		24

void _init()
{
	wiringPiSetup();

	sleep(5);

	pinMode(In1, OUTPUT);
	pinMode(In2, OUTPUT);
	pinMode(En1, OUTPUT);
	pinMode(In3, OUTPUT);
	pinMode(In4, OUTPUT);
	pinMode(En2, PWM_OUTPUT);

	digitalWrite(In1, 0);
	digitalWrite(In2, 0);
	digitalWrite(En1, 0);
	digitalWrite(In3, 0);
	digitalWrite(In4, 0);
	pwmWrite(En2, 0);

	sleep(10);

	ROS_INFO_STREAM("GPIO initialization completed");
}

int cmd_vel_lin_conv(const geometry_msgs::Twist& msg)
{
	int throttle;
	
	if(msg.linear.x >= 0.4) { throttle = 1; return throttle;}
	if(msg.linear.x <= -0.4) { throttle = -1; return throttle;}
	else { throttle = 0; return throttle;}
}

int cmd_vel_ang_conv(const geometry_msgs::Twist& msg)
{
	int stearing;
	
	if(msg.angular.z >= 0.4) { stearing = 1; return stearing;}
	if(msg.angular.z <= -0.4) { stearing = -1; return stearing;}
	else { stearing = 0; return stearing;}
}

static void cmd_vel_act(const geometry_msgs::Twist& msg)
{
	int throttle;
	int stearing;
	throttle = cmd_vel_lin_conv(msg);
	stearing = cmd_vel_ang_conv(msg);

	if (throttle == 0)
	{
		digitalWrite(In1, 0);
		digitalWrite(In2, 0);
		digitalWrite(En1, 1);
	}
	
	if (throttle == 1)
	{
		digitalWrite(In1, 1);
		digitalWrite(In2, 0);
		digitalWrite(En1, 1);
	}
	
	if (throttle == -1)
	{
		digitalWrite(In1, 0);
		digitalWrite(In2, 1);
		digitalWrite(En1, 1);
	}

	if (stearing == 0)
	{
		digitalWrite(In3, 0);
		digitalWrite(In4, 0);
		pwmWrite(En2, 0);
	}

	if (stearing == 1)
	{
		digitalWrite(In3, 1);
		digitalWrite(In4, 0);
		pwmWrite(En2, 896);
	}

	if (stearing == -1)
	{
		digitalWrite(In3, 0);
		digitalWrite(In4, 1);
		pwmWrite(En2, 896);
	}
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "l298n_drive");

	ros::NodeHandle nh;

	_init();

	ros::Subscriber sub = nh.subscribe("/cmd_vel", 500, cmd_vel_act);
	
	ROS_INFO_STREAM("Lego Car READY!");

	ros::spin();
	return 0;
} 

