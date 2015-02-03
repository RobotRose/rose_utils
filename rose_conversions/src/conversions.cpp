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
#include "rose_conversions/conversions.hpp"

namespace rose_conversions { 

string ReplaceString(string subject, const string& search, const string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
    return subject;
}

string IntToString(int number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

string floatToString(float number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}


string doubleToString(double number)
{
   stringstream ss;//create a stringstream
   ss << number;//add number to the stream
   return ss.str();//return a string with the contents of the stream
}

int StringToInt(string number_string)
{
    int x = 0;
    try {
        x = boost::lexical_cast<int>(number_string);
    } catch( boost::bad_lexical_cast const& ) {
        throw;
    }
    return x;
}

double StringToDouble(string number_string)
{
    double x = 0;
    try {
        x = boost::lexical_cast<double>(number_string);
    } catch( boost::bad_lexical_cast const& ) {
        throw;
    }
    return x;
}

double floatToDouble( float x )
{
    return (double)x;
}

float doubleToFloat( double x )
{
    return (float)x;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

// Convert quaternion to RPY
geometry_msgs::Vector3 quaternionToRPY(const geometry_msgs::Quaternion& quat)
{
    geometry_msgs::Vector3 vector;
    tf::Quaternion q;
    tf::quaternionMsgToTF(quat, q);
    return quaternionToRPY(q);
}

geometry_msgs::Vector3 quaternionToRPY(const tf::Quaternion& quat)
{
    geometry_msgs::Vector3 vector;
    tf::Matrix3x3(quat).getRPY(vector.x, vector.y, vector.z);
    return vector;
}

geometry_msgs::Quaternion RPYToQuaterion( const geometry_msgs::Vector3& rpy)
{
    return tf::createQuaternionMsgFromRollPitchYaw(rpy.x, rpy.y, rpy.z);;
}

geometry_msgs::Quaternion RPYToQuaterion( const double& roll, const double& pitch, const double& yaw)
{
    return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

const char* byteToBinary(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

geometry_msgs::Point point32ToPoint( const geometry_msgs::Point32 point32 )
{
    geometry_msgs::Point point;
    point.x = floatToDouble(point32.x);
    point.y = floatToDouble(point32.y);
    point.z = floatToDouble(point32.z);

    return point;
}

geometry_msgs::Point32 pointToPoint32( const geometry_msgs::Point point )
{
    geometry_msgs::Point32 point32;
    point32.x = doubleToFloat(point.x);
    point32.y = doubleToFloat(point.y);
    point32.z = doubleToFloat(point.z);

    return point32;
}


} // rose_conversions