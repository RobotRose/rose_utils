/***********************************************************************************
* Copyright: Rose B.V. (2015)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2015/02/03
* 		- File created.
*
* Description:
*	Conversions
* 
***********************************************************************************/
#ifndef CONVERSIONS_HPP
#define CONVERSIONS_HPP

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <termios.h>

#include <tf/transform_listener.h> 

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include "rose_geometry/point.hpp"
#include "rose_geometry/geometry.hpp"

using geometry_msgs::Point32;
using geometry_msgs::Point;

using std::string;
using std::stringstream;

namespace rose_conversions { 

string  replaceString(string subject, const string& search, const string& replace);
string  intToString(int number);

string  floatToString(float number);
string  doubleToString(double number);

int     stringToInt(string number_string);
double  stringToDouble(string number_string);

double  floatToDouble( float x );
float   doubleToFloat( double x );

int     kbhit();

geometry_msgs::Vector3      quaternionToRPY(const geometry_msgs::Quaternion& quat);
geometry_msgs::Vector3      quaternionToRPY(const tf::Quaternion& quat);
geometry_msgs::Quaternion   RPYToQuaterion( const geometry_msgs::Vector3& rpy);
geometry_msgs::Quaternion   RPYToQuaterion( const double& roll, const double& pitch, const double& yaw);

Point           point32ToPoint( const Point32 point32 );
Point32         pointToPoint32( const Point point );

const char* byteToBinary(int x);

//! @todo MdL [QSTN]: Move to other package?
bool getRampedVelocity(  const double& min_velocity, 
                         const double& max_velocity, 
                         const double& min_dist_to_goal, 
                         const double& max_dist_velocity,
                         const double& distance_to_goal,
                               double& velocity );

bool getRampedVelocity(  const double& min_velocity, 
                         const double& max_velocity, 
                         const double& min_dist_to_goal, 
                         const double& max_dist_velocity,
                         const rose_geometry::Point& distance_to_goal,
                               rose_geometry::Point& velocity);

bool limit(float min, float max, float* number);
bool limit(double min, double max, double* number);
bool limit(int min, int max, int* number);

template <typename T> 
int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

} // rose_conversions

#endif // CONVERSIONS_HPP