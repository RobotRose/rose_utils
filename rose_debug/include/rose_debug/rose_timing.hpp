/***********************************************************************************
* Copyright: Rose B.V. (2015)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2015/03/10
*       - File created.
*
* Description:
*   Rose timing class, to time runtime
* 
***********************************************************************************/
#ifndef ROSE_TIMING_HPP
#define ROSE_TIMING_HPP

#include <ros/ros.h>

namespace rose_debug{

class Timing
{
public:
    Timing(const std::string& name, const int& average_over);
    ~Timing();
    void show(const float& rate = 0.0);
    void start();
    void stop();
    void reset();

private:
    void purge();

private:
    std::string             name_;
    int                     average_over_;
    std::vector<ros::Time>  start_times;
    std::vector<ros::Time>  finish_times;

};
} // namespace rose_debug

#endif // ROSE_TIMING_HPP 
