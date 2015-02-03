/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/07/25
* 		- File created.
*
* Description:
*	Rose point class
* 
***********************************************************************************/

#ifndef POINT_HPP
#define POINT_HPP

#include <iostream>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"

namespace rose_geometry{

class Point
{
  public:
	Point();
	Point(float init_x, float init_y , float init_z = 0.0);
	Point(const geometry_msgs::Point& point);
	~Point();

	// Conversion functions
	geometry_msgs::Point getROSmsg() const;
	geometry_msgs::Point32 getROSmsg32() const;

	// Linear algebra functions
	float dot(const Point& rhs);
	float cross_2d(const Point& rhs);

	// Operators
	inline Point& operator=(const geometry_msgs::Point& rhs)
	{
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
		return *this;
	}

	inline Point& operator+=(const Point& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
		return *this;
	}

	inline Point& operator-=(const Point& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
		return *this;
	}

	inline Point& operator*=(const Point& rhs)
	{
		x *= rhs.x;
		y *= rhs.y;
		z *= rhs.z;
		return *this;
	}

	inline Point& operator/=(const Point& rhs)
	{
		x /= rhs.x;
		y /= rhs.y;
		z /= rhs.z;
		return *this;
	}



	inline Point& operator*=(const float& rhs)
	{
		x *= rhs;
		y *= rhs;
		z *= rhs;
		return *this;
	}

	inline Point& operator+=(const float& rhs)
	{
		x += rhs;
		y += rhs;
		z += rhs;
		return *this;
	}

	inline Point& operator-=(const float& rhs)
	{
		x -= rhs;
		y -= rhs;
		z -= rhs;
		return *this;
	}

	inline Point& operator/=(const float& rhs)
	{
		x /= rhs;
		y /= rhs;
		z /= rhs;
		return *this;
	}

	float x;
	float y;
	float z;
  protected:
  private:
};
 
// Non-member Point type operators
inline Point operator+(Point lhs, const Point& rhs)
{
  lhs += rhs;
  return lhs;
}

inline Point operator-(Point lhs, const Point& rhs)
{
  lhs -= rhs;
  return lhs;
}

inline Point operator*(Point lhs, const Point& rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Point operator/(Point lhs, const Point& rhs)
{
  lhs /= rhs;
  return lhs;
}

inline Point operator+(Point lhs, const float& rhs)
{
  lhs += rhs;
  return lhs;
}

inline Point operator-(Point lhs, const float& rhs)
{
  lhs -= rhs;
  return lhs;
}

inline Point operator*(Point lhs, const float& rhs)
{
  lhs *= rhs;
  return lhs;
}

inline Point operator/(Point lhs, const float& rhs)
{
  lhs /= rhs;
  return lhs;
}

inline Point operator+(const float lhs, Point rhs)
{
  rhs += lhs;
  return rhs;
}

inline Point operator-(const float lhs, Point rhs)
{
  rhs -= lhs;
  return rhs;
}

inline Point operator*(const float lhs, Point rhs)
{
  rhs *= lhs;
  return rhs;
}

inline Point operator/(const float lhs, Point rhs)
{
  rhs /= lhs;
  return rhs;
}

inline std::ostream& operator<<(std::ostream& os, const Point& p) {
    return os << "[" << p.x << ", " << p.y << ", " << p.z << "]";
}

inline bool operator==(const Point& lhs, const Point& rhs){return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z); }
inline bool operator!=(const Point& lhs, const Point& rhs){return !operator==(lhs,rhs);}
inline bool operator< (const Point& lhs, const Point& rhs){return false; }
inline bool operator> (const Point& lhs, const Point& rhs){return  operator< (rhs,lhs);}
inline bool operator<=(const Point& lhs, const Point& rhs){return !operator> (lhs,rhs);}
inline bool operator>=(const Point& lhs, const Point& rhs){return !operator< (lhs,rhs);}

// Non-member conversion functions
Point fromROSmsg(const geometry_msgs::Point& geometry_msgs_point);


// Non-member functions
float distance(const Point& a, const Point& b);
float distanceSq(const Point& a, const Point& b);
float angle(const Point& a, const Point& b);
float angle(const float& radians, const Point& b);
float angle(const Point& a, const float& radians);
std::vector<geometry_msgs::Point> 	toROSmsgs(const std::vector<Point>& points);
std::vector<geometry_msgs::Point32> toROSmsgs32(const std::vector<Point>& points);

}; // rose_geometry

#endif // POINT_HPP
