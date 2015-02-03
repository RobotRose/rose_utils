/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/07/23
* 		- File created.
*
* Description:
*	Twist moving average filter
* 
***********************************************************************************/
#include "rose_twist_moving_average_filter/twist_maf.hpp"

TwistMAF::TwistMAF()
    : window_size_(1),
    linear_x_acc_(tag::rolling_window::window_size = 1),
    linear_y_acc_(tag::rolling_window::window_size = 1),
    linear_z_acc_(tag::rolling_window::window_size = 1),
    angular_x_acc_(tag::rolling_window::window_size = 1),
    angular_y_acc_(tag::rolling_window::window_size = 1),
    angular_z_acc_(tag::rolling_window::window_size = 1)
{}

TwistMAF::~TwistMAF()
{}

// This reset the wegihted average!
bool TwistMAF::setWindowSize(const unsigned int n)
{
    if(n == 0)
        return false;

    window_size_ = n;

    linear_x_acc_   = accumulator(tag::rolling_window::window_size = window_size_);
    linear_y_acc_   = accumulator(tag::rolling_window::window_size = window_size_);
    linear_z_acc_   = accumulator(tag::rolling_window::window_size = window_size_);
    angular_x_acc_  = accumulator(tag::rolling_window::window_size = window_size_);
    angular_y_acc_  = accumulator(tag::rolling_window::window_size = window_size_);
    angular_z_acc_  = accumulator(tag::rolling_window::window_size = window_size_);
    return true;
}

int TwistMAF::getWindowSize() const
{
    return window_size_;
}

Twist TwistMAF::getMovingAverage() const
{
    Twist moving_average_twist;

    moving_average_twist.linear.x  = rolling_mean(linear_x_acc_);
    moving_average_twist.linear.y  = rolling_mean(linear_y_acc_);
    moving_average_twist.linear.z  = rolling_mean(linear_z_acc_);
    moving_average_twist.angular.x = rolling_mean(angular_x_acc_);
    moving_average_twist.angular.y = rolling_mean(angular_y_acc_);
    moving_average_twist.angular.z = rolling_mean(angular_z_acc_);

    return moving_average_twist;
}

void TwistMAF::operator()(const Twist& twist)
{
    // Adding sample to rolling average filters
    linear_x_acc_(twist.linear.x);
    linear_y_acc_(twist.linear.y);
    linear_z_acc_(twist.linear.z);
    angular_x_acc_(twist.angular.x);
    angular_y_acc_(twist.angular.y);
    angular_z_acc_(twist.angular.z);
}

void TwistMAF::reset()
{
    linear_x_acc_   = accumulator(tag::rolling_window::window_size = window_size_);
    linear_y_acc_   = accumulator(tag::rolling_window::window_size = window_size_);
    linear_z_acc_   = accumulator(tag::rolling_window::window_size = window_size_);
    angular_x_acc_  = accumulator(tag::rolling_window::window_size = window_size_);
    angular_y_acc_  = accumulator(tag::rolling_window::window_size = window_size_);
    angular_z_acc_  = accumulator(tag::rolling_window::window_size = window_size_);
}