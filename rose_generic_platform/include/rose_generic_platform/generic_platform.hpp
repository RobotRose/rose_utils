/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Okke Hendriks
*   Date  : 2014/08/04
*       - File created.
*
* Description:
*   This is the header file of the GenericPlatform class, a generic platform descriptor.
* 
***********************************************************************************/
#ifndef GENERIC_PLATFORM_HPP
#define GENERIC_PLATFORM_HPP

#include <stdio.h>
#include <iostream>

#include <ros/ros.h>

#include "rose_common/common.hpp"
#include "rose20_common/wheel_unit.hpp"
#include "rose_geometry/point.hpp"

class GenericPlatform
{
  public:
    GenericPlatform();
    GenericPlatform(string name);
    ~GenericPlatform();

    const std::vector<rose_geometry::Point>&  getFootprint() const;
    bool                                                setFootprint(const std::vector<rose_geometry::Point>& new_footprint);
    float   getMaxVelocity(string direction) const;
    bool    setMaxVelocity(string direction, float max_velocity);
    float   getMinVelocity(string direction) const;
    bool    setMinVelocity(string direction, float min_velocity);
    const std::vector<WheelUnit>&   getWheelUnits() const;
    bool                            setWheelUnits(const std::vector<WheelUnit>& wheel_units);
    bool                            addWheelUnit(const rose_geometry::Point& position, const WheelUnit& wheel_unit);

  protected:
  private:
    string                                          name_;
    std::vector<rose_geometry::Point>     footprint_;
    std::vector<WheelUnit>                          wheelunits_;
    std::map<string, float>                         max_velocity_map_;
    std::map<string, float>                         min_velocity_map_;

};

#endif // GENERIC_PLATFORM_HPP
