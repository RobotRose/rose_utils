/***********************************************************************************
* Copyright: Rose B.V. (2015)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2015/03/10
* 		- File created.
*
* Description:
*	Rose timing class
* 
***********************************************************************************/

#include "rose_debug/rose_timing.hpp"

namespace rose_debug
{

Timing::Timing(const std::string& name, const int& average_over)
    : name_(name)
    , average_over_(average_over)
{}

Timing::~Timing()
{}

void Timing::show(const float& rate)
{
    purge();

    if( start_times.size() != finish_times.size())
    {
        ROS_ERROR("Timing '%s': The number of start times is not equal to the number of stop times.", name_.c_str());
    }

    float average = 0;
    int samples = start_times.size();
    for(int i = 0; i < samples; i++)
    {
        average += finish_times.at(i).toSec() - start_times.at(i).toSec();
    }

    average /= (float)samples;


    if(rate != 0.0)
        ROS_INFO_THROTTLE(rate, "Timing '%s', timing over %d samples, average: %.6f", name_.c_str(), samples, average);
    else
        ROS_INFO("Timing '%s', timing over %d samples, average: %.6f", name_.c_str(), samples, average);
}

void Timing::start()
{
    start_times.push_back(ros::Time::now());
}

void Timing::stop()
{
    finish_times.push_back(ros::Time::now());
}

void Timing::reset()
{
    start_times.clear();
    finish_times.clear();
}

void Timing::purge()
{
    if(start_times.size() > average_over_)
        start_times.erase(start_times.begin(), std::next(start_times.begin(), start_times.size() - average_over_ - 1 ));

    if(finish_times.size() > average_over_)
        finish_times.erase(finish_times.begin(), std::next(finish_times.begin(), finish_times.size() - average_over_ - 1 ));
}
} // namespace timing
