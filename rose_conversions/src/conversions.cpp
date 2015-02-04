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

string replaceString(string subject, const string& search, const string& replace) {
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
    }
    return subject;
}

string intToString(int number)
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

int stringToInt(string number_string)
{
    int x = 0;
    try {
        x = boost::lexical_cast<int>(number_string);
    } catch( boost::bad_lexical_cast const& ) {
        throw;
    }
    return x;
}

double stringToDouble(string number_string)
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

bool getRampedVelocity(  const double& min_velocity, 
                         const double& max_velocity, 
                         const double& min_dist_to_goal, 
                         const double& max_dist_velocity,
                         const double& distance_to_goal,
                               double& velocity )
{
    ROS_DEBUG("getRampedVelocity: minvel: %f, maxvel: %f, mindisttogoal: %f, max_dist_vel: %f, distance to goal: %f", 
        min_velocity, 
        max_velocity, 
        min_dist_to_goal, 
        max_dist_velocity,
        distance_to_goal);

    if (std::fabs(distance_to_goal) < min_dist_to_goal)
    {
        velocity = min_velocity;
        return true;
    }

    velocity = ((max_velocity - min_velocity)/max_dist_velocity)*(distance_to_goal-min_dist_to_goal);
    ROS_DEBUG("calculated velocity = %f", velocity);
    // limit max
    limit(-max_velocity, max_velocity, &velocity);

    // limit min
    if(std::fabs(velocity) < min_velocity)
        velocity = sgn(velocity) * min_velocity;

    ROS_DEBUG("end velocity = %f", velocity);

    return true;
}

bool getRampedVelocity(  const double& min_velocity, 
                         const double& max_velocity, 
                         const double& min_dist_to_goal, 
                         const double& max_dist_velocity,
                         const rose20_common::geometry::Point& distance_to_goal,
                               rose20_common::geometry::Point& velocity)
{
    ROS_DEBUG("getRampedVelocity: minvel: %f, maxvel: %f, mindisttogoal: %f, max_dist_vel: %f, distance to goal: (%f,%f)", 
        min_velocity, 
        max_velocity, 
        min_dist_to_goal, 
        max_dist_velocity,
        distance_to_goal.x,
        distance_to_goal.y);

    if (rose20_common::getVectorLengthXY(distance_to_goal.x, distance_to_goal.y) < min_dist_to_goal)
    {
        velocity = distance_to_goal;
        rose20_common::setVectorLengthXY(&velocity.x, &velocity.y, min_velocity);
        return true;
    }

    rose20_common::setVectorLengthXY(&velocity.x, &velocity.y, rose20_common::getVectorLengthXY(distance_to_goal.x, distance_to_goal.y)-min_dist_to_goal);

    velocity.x = ((max_velocity - min_velocity)/max_dist_velocity)*distance_to_goal.x;
    velocity.y = ((max_velocity - min_velocity)/max_dist_velocity)*distance_to_goal.y;
    ROS_DEBUG("calculated velocity = (%f,%f)", velocity.x, velocity.y);

    // limit max
     rose20_common::limitVectorLengthXY(&velocity.x, &velocity.y, max_velocity);

    // Limit min
    if(rose20_common::getVectorLengthXY(velocity.x, velocity.y) < min_velocity)
        rose20_common::setVectorLengthXY(&velocity.x, &velocity.y, min_velocity);

    ROS_DEBUG("end velocity = (%f,%f)", velocity.x, velocity.y);
    return true;
}


} // rose_conversions