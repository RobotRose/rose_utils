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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;

using std::string;
using std::stringstream;

namespace rose_conversions { 

string  ReplaceString(string subject, const string& search, const string& replace);
string  IntToString(int number);

string  floatToString(float number);
string  doubleToString(double number);

int     StringToInt(string number_string);
double  StringToDouble(string number_string);

double  floatToDouble( float x );
float   doubleToFloat( double x );

int     kbhit();

geometry_msgs::Vector3      quaternionToRPY(const geometry_msgs::Quaternion& quat);
geometry_msgs::Vector3      quaternionToRPY(const tf::Quaternion& quat);
geometry_msgs::Quaternion   RPYToQuaterion( const geometry_msgs::Vector3& rpy);
geometry_msgs::Quaternion   RPYToQuaterion( const double& roll, const double& pitch, const double& yaw);

Point           point32ToPoint( const Point32 point32 );
Point32         pointToPoint32( const Point point );

} // rose_conversions

#endif // CONVERSIONS_HPP