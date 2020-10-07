#include <wiringPi.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmath>
#include <ctime>
#include <iostream>

// define gpio channels for each input required by the l298n 
#define In1		15
#define In2		14
#define En1		18
#define In3		16
#define In4		20
#define En2		19

 // +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
 // | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 // +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 // |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 // |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
 // |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
 // |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | OUT  | TxD     | 15  | 14  |
 // |     |     |      0v |      |   |  9 || 10 | 1 | OUT  | RxD     | 16  | 15  |
 // |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 1 | ALT5 | GPIO. 1 | 1   | 18  |
 // |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 // |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
 // |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 // |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
 // |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 // |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
 // |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
 // |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 // |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 // |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
 // |  13 |  23 | GPIO.23 |   IN | 1 | 33 || 34 |   |      | 0v      |     |     |
 // |  19 |  24 | GPIO.24 | ALT5 | 0 | 35 || 36 | 0 | OUT  | GPIO.27 | 27  | 16  |
 // |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | OUT  | GPIO.28 | 28  | 20  |
 // |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
 // +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 // | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 // +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+

// function for gpio initialization
void _init()
{
	wiringPiSetupGpio();

	pinMode(In1, OUTPUT);
	pinMode(In2, OUTPUT);
	pinMode(En1, PWM_OUTPUT);
	pinMode(In3, OUTPUT);
	pinMode(In4, OUTPUT);
	pinMode(En2, PWM_OUTPUT);

	pwmSetMode(PWM_MODE_MS);	//mark space pwm mode, it is the classic way to intend a pwm signal, other option is balanced
	//pwmSetClock(20);	// divide the base clock frequency for a denominator

	digitalWrite(In1, 0);
	digitalWrite(In2, 0);
    pwmWrite(En1, 0);
	digitalWrite(In3, 0);
	digitalWrite(In4, 0);
	pwmWrite(En2, 0);

	ROS_INFO_STREAM("GPIO initialization completed");
}

// function for extrapolation of the linear value from the cmd_vel message
double cmd_vel_lin_conv(const geometry_msgs::Twist& msg)
{	
	return msg.linear.x/0.5 * 1024;
}

// function for extrapolation of the angular value from the cmd_vel message
double cmd_vel_ang_conv(const geometry_msgs::Twist& msg)
{
	return msg.angular.z/0.5 * (832);
}

static void cmd_vel_act(const geometry_msgs::Twist& msg)
{  	
	double throttle;
	double stearing;
	throttle = cmd_vel_lin_conv(msg);
	stearing = cmd_vel_ang_conv(msg);
	
	
	if(throttle <= 128) 	// brake
	{
		digitalWrite(In1, 1);
		digitalWrite(In2, 1);
		pwmWrite(En1, 1024);
	}

	if (throttle > 128)		// backward
	{
		digitalWrite(In1, 1);
		digitalWrite(In2, 0);
		pwmWrite(En1, (int)throttle);
	}
	
	if (throttle < -128)	// forward
	{
		throttle = abs(throttle);
		digitalWrite(In1, 0);
		digitalWrite(In2, 1);
		pwmWrite(En1, (int)throttle);
	}

	if (abs(stearing) <= 10)	// stearing to the center
	{
		digitalWrite(In3, 0);
		digitalWrite(In4, 0);
		digitalWrite(En2, 1);
	}

	if (stearing > 10)		// turn left
	{
		digitalWrite(In3, 0);
		digitalWrite(In4, 1);
		pwmWrite(En2, (int)stearing);
	}

	if (stearing < -10)		// turn right
	{
		stearing = abs(stearing);
		digitalWrite(In3, 1);
		digitalWrite(In4, 0);
		pwmWrite(En2, (int)stearing);
	}
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "l298n_drive");

	ros::NodeHandle nh;

	_init();

	ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, cmd_vel_act);

	ROS_INFO_STREAM("Lego Car READY!");

	ros::spin();

	return 0;
} 

