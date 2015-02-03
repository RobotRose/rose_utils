#include "PID/PID.hpp"

PID::PID()
{
	initialize(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}


PID::PID(float Kp, float Ki, float Kd, float i_min, float i_max, float cmd_min, float cmd_max)
{
	initialize(Kp, Ki, Kd, i_min, i_max, cmd_min, cmd_max);
}

PID::~PID()
{}

void PID::initialize(float Kp, float Ki, float Kd, float i_min, float i_max, float cmd_min, float cmd_max)
{
	Kp_			= Kp;
	Ki_			= Ki;
	Kd_			= Kd;
	integral_ 	= 0.0;
	derivative_ = 0.0;
	output_ 	= 0.0;
	i_min_ 		= i_min;
	i_max_ 		= i_max;
	cmd_min_	= cmd_min;
	cmd_max_ 	= cmd_max;
	prev_error_ = 0.0;
	prev_time_ 	= ros::Time::now();
}
float PID::update(float error)
{
	// Calculate time difference
	float dt = ros::Time::now().toSec() - prev_time_.toSec();
	
	// Check time difference
	if(dt <= 0.0)
	{
		ROS_WARN_NAMED(ROS_NAME_PID, "Invalid dt: %f", dt);
		prev_time_ 	= ros::Time::now();
		return 0.0;
	}

	return update(error, dt);
}

float PID::update(float error, float dt)
{
	// Calculate integral
	integral_   = integral_ + (error*dt);

	// Limit integral
	integral_ 	= fmin(i_max_, fmax(i_min_, integral_));

	// Calculate derivative
	derivative_ = (error - prev_error_)/dt;
	
	// Calculate output
	float P 	= (Kp_*error);
	float I 	= (Ki_*integral_);
	float D 	= (Kd_*derivative_);

	output_ 	= P + I + D;

	// Limit output
	output_ 	= fmin(cmd_max_, fmax(cmd_min_, output_));

	ROS_DEBUG_NAMED(ROS_NAME_PID, "PID:[%.4f, %.4f, %.4f] -> %.4f", P, I, D, output_);

	// Store state
    prev_error_ = error;
	prev_time_ 	= ros::Time::now();

	// Return control output
	return output_;
}
