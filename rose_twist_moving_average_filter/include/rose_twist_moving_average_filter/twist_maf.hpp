/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/07/23
*       - File created.
*
* Description:
*   Twist message moving average filter
* 
***********************************************************************************/
#ifndef TWIST_MAF_HPP
#define TWIST_MAF_HPP

#include <ros/ros.h>

#include <iostream>

#include <geometry_msgs/Twist.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

using namespace boost::accumulators;
using geometry_msgs::Twist;

class TwistMAF
{
  public:
    TwistMAF();
    ~TwistMAF();

    bool    setWindowSize(const unsigned int n);
    int     getWindowSize() const;
    Twist   getMovingAverage() const;
    void    operator()(const Twist& twist);
    void    reset();

  private:
    typedef accumulator_set<float, stats<tag::rolling_mean> > accumulator;
    accumulator linear_x_acc_;
    accumulator linear_y_acc_;
    accumulator linear_z_acc_;
    accumulator angular_x_acc_;
    accumulator angular_y_acc_;
    accumulator angular_z_acc_;

    unsigned int window_size_;
};

#endif // TWIST_MAF_HPP