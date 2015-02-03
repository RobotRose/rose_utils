/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/03/07
* 		- File created.
*
* Description:
*	PID controller class
* 
***********************************************************************************/

#ifndef PID_HPP
#define PID_HPP

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>

#include "rose_common/common.hpp"

#define ROS_NAME_PID    ROS_NAME + "|PID"

using namespace std;

class PID{
  public:
  	PID();
  	PID(float Kp, float Ki, float Kd, float i_min, float i_max, float cmd_min, float cmd_max);
  	~PID();

    void initialize(float Kp, float Ki, float Kd, float i_min, float i_max, float cmd_min, float cmd_max);
  	float update(float error);  
    float update(float error, float dt);    	

  private:
  	float Kp_;
  	float Ki_;
  	float Kd_;
  	float i_min_;
    float i_max_;
    float cmd_min_;
    float cmd_max_;

  	float integral_;
  	float derivative_;
  	float output_;

  	float 		prev_error_;
  	ros::Time 	prev_time_;
};

#endif // PID_HPP
