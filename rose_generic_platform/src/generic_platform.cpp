/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/08/04
* 		- File created.
*
* Description:
*	The implementation of the GenericPlatform class.
* 
***********************************************************************************/
#include "rose_generic_platform/generic_platform.hpp"

GenericPlatform::GenericPlatform()
{}

GenericPlatform::GenericPlatform(string name)
	: name_(name)
{
	//! @todo OH: Load configuration of the platform from ROS param's (yaml files) For now just initialize by name

	if(name_ == "rose20")
	{
	    // Initialize all maximal and minimal values, and other properties
	    setMaxVelocity("X", 0.15);
	    setMaxVelocity("Y", 0.15);
	    setMaxVelocity("XY_ABS", sqrt(pow(getMaxVelocity("X"), 2.0) + pow(getMaxVelocity("Y"), 2.0)));
	    setMaxVelocity("THETA", 0.2);
	    setMaxVelocity("THETA_INPLACE", 0.5);

	    setMinVelocity("X", 0.05);
	    setMinVelocity("Y", 0.05);
	    setMinVelocity("XY_ABS", 0.05);
	    setMinVelocity("THETA", 0.05);
	    setMinVelocity("THETA_INPLACE", 0.15);
	}
}

GenericPlatform::~GenericPlatform()
{}

const std::vector<rose_geometry::Point>& GenericPlatform::getFootprint() const
{
	return footprint_;
}

bool GenericPlatform::setFootprint(const std::vector<rose_geometry::Point>& new_footprint)
{
	footprint_ = new_footprint;
	return true;
}

float GenericPlatform::getMaxVelocity(string direction) const
{
	const auto& iterator = max_velocity_map_.find(direction);
	
	if(iterator != max_velocity_map_.end())
		return max_velocity_map_.at(direction);
	else
	{
		ROS_WARN_NAMED(ROS_NAME, "No maximal velocity stored under '%s'.", direction.c_str());
		return 0.0;
	}
}

bool GenericPlatform::setMaxVelocity(string direction, float max_velocity)
{
	max_velocity_map_[direction] = max_velocity;
	return true;
}

float GenericPlatform::getMinVelocity(string direction) const
{
	const auto& iterator = min_velocity_map_.find(direction);
	
	if(iterator != min_velocity_map_.end())
		return min_velocity_map_.at(direction);
	else
	{
		ROS_WARN_NAMED(ROS_NAME, "No minimal velocity stored under '%s'.", direction.c_str());
		return 0.0;
	}
}

bool GenericPlatform::setMinVelocity(string direction, float min_velocity)
{
	min_velocity_map_[direction] = min_velocity;
	return true;
}

const std::vector<WheelUnit>& GenericPlatform::getWheelUnits() const
{
	return wheelunits_;
}

bool GenericPlatform::setWheelUnits(const std::vector<WheelUnit>& wheel_units)
{
	wheelunits_ = wheel_units;
	return true;
}

bool GenericPlatform::addWheelUnit(const rose_geometry::Point& position, const WheelUnit& wheel_unit)
{
	wheelunits_.push_back(wheel_unit);
	return true;
}

