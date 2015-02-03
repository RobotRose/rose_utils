/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/07/25
* 		- File created.
*
* Description:
*	Rose ROS point class
* 
***********************************************************************************/
#include "rose_geometry/point.hpp"

namespace rose_geometry{

Point::Point()
	: x(0.0),
	  y(0.0),
	  z(0.0)
{}

Point::Point(float init_x, float init_y, float init_z)
{
	x = init_x;
	y = init_y;
	z = init_z;
}

Point::Point(const geometry_msgs::Point& point)
{
	x = point.x;
	y = point.y;
	z = point.z;
}

Point::~Point()
{}

// Conversion functions
geometry_msgs::Point Point::getROSmsg() const
{
	geometry_msgs::Point point;
	point.x = x;
	point.y = y;
	point.z = z;
	return point;
}

geometry_msgs::Point32 Point::getROSmsg32() const
{
	geometry_msgs::Point32 point;
	point.x = x;
	point.y = y;
	point.z = z;
	return point;
}

// Linear algebra functions 
float Point::dot(const Point& rhs)
{
	return x*rhs.x + y*rhs.y + z*rhs.z;
}

float Point::cross_2d(const Point& rhs)
{
	return x*rhs.y - y*rhs.x;
}

//! @todo OH: make unit test for this function
Point fromROSmsg(const geometry_msgs::Point& geometry_msgs_point)
{
	return Point(geometry_msgs_point);
}

float distance(const Point& a, const Point& b)
{

	return sqrt(distanceSq(a, b));
}

float distanceSq(const Point& a, const Point& b)
{
	rose_geometry::Point diff_vector = b - a;
	return (diff_vector.x*diff_vector.x) + (diff_vector.y*diff_vector.y);
}

float angle(const Point& a, const Point& b)
{
    return atan2((b - a).y, (b - a).x);
}

float angle(const float& radians, const Point& b)
{
	Point a(1.0, 0.0, 0.0);

    float x_temp = a.x;
	float y_temp = a.y;
	a.x = x_temp*cos(radians) - y_temp*sin(radians);
	a.y = y_temp*cos(radians) + x_temp*sin(radians);

   	return angle(a, b);
}

float angle(const Point& a, const float& radians)
{
	return angle(radians, a);
}

std::vector<geometry_msgs::Point> toROSmsgs(const std::vector<Point>& points)
{
	std::vector<geometry_msgs::Point> ros_points;
	for(const auto& point : points)
		ros_points.push_back(point.getROSmsg());
	return ros_points;
}

std::vector<geometry_msgs::Point32> toROSmsgs32(const std::vector<Point>& points)
{
	std::vector<geometry_msgs::Point32> ros_points;
	for(const auto& point : points)
		ros_points.push_back(point.getROSmsg32());
	return ros_points;
}

}	// namespace rose_geometry
