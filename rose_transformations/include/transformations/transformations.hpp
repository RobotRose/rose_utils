/***********************************************************************************
* Copyright: Rose B.V. (2015)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2015/02/03
* 		- File created.
*
* Description:
*	Transformations
* 
***********************************************************************************/
#ifndef TRANSFORMATIONS_HPP
#define TRANSFORMATIONS_HPP

#include <tf/transform_listener.h> 

#include "rose_geometry/point.hpp"
#include "rose_geometry/stamped.hpp"

#include "ros_name/ros_name.hpp"

using geometry_msgs::Point;
using geometry_msgs::Point32;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;

namespace rose_transformations { 
/**
 * Transform PointStamped to a particular frame. It is transformed in the time stamp set in the header of the point.
 * @param  tf      Transformlistener
 * @param  frame   Which frame to transform to
 * @param  point   Point that should be transformed
 * @param  timeout How long to wait for the frame to be available
 * @return         If the transform was succesful
 */
bool transformToFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PointStamped& point, const double timeout = 10.0);

/**
 * Transform PoseStamped to a particular frame. It is transformed in the time stamp set in the header of the pose.
 * @param  tf      Transformlistener
 * @param  frame   Which frame to transform to
 * @param  pose    Pose that should be transformed
 * @param  timeout How long to wait for the frame to be available
 * @return         If the transform was succesful
 */
bool transformToFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PoseStamped& pose, const double timeout = 10.0);

bool transformToFrame( const tf::TransformListener& tf, const std::string frame, sensor_msgs::PointCloud& pose, const double timeout = 10.0);

/**
 * Transform PointStamped to a particular frame. It is transformed to the frame that was available latest in time.
 * @param  tf      Transformlistener
 * @param  frame   Which frame to transform to
 * @param  point   Point that should be transformed
 * @param  timeout How long to wait for the frame to be available
 * @return         If the transform was succesful
 */
bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PointStamped& point, const double timeout = 10.0);

/**
 * Transform PoseStamped to a particular frame. It is transformed to the frame that was available latest in time.
 * @param  tf      Transformlistener
 * @param  frame   Which frame to transform to
 * @param  pose    Pose that should be transformed
 * @param  timeout How long to wait for the frame to be available
 * @return         If the transform was succesful
 */
bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, const rose_geometry::Stamped<rose_geometry::Point>& stamped_point, const double timeout = 10.0);
bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PoseStamped& pose, const double timeout = 10.0);
bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, sensor_msgs::PointCloud& pose, const double timeout = 10.0);
//! @todo OH: Should these have a timeout? Its latest, they should have max_age as getLatestFrameInFrame?

/**
 * Transform PointStamped to a particular frame. It is transformed to the frame that was available now.
 * @param  tf      Transformlistener
 * @param  frame   Which frame to transform to
 * @param  point   Point that should be transformed
 * @param  timeout How long to wait for the frame to be available
 * @return         If the transform was succesful
 */
bool transformToFrameNow( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PointStamped& point, const double timeout = 10.0);

/**
 * Transform PoseStamped to a particular frame. It is transformed to the frame that was available now.
 * @param  tf      Transformlistener
 * @param  frame   Which frame to transform to
 * @param  pose    Pose that should be transformed
 * @param  timeout How long to wait for the frame to be available
 * @return         If the transform was succesful
 */
bool transformToFrameNow( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PoseStamped& pose, const double timeout = 10.0);

bool transformToFrameNow( const tf::TransformListener& tf, const std::string frame, sensor_msgs::PointCloud& pose, const double timeout = 10.0);

//! @todo OH: Doxygen comment.
bool getFrameInFrame( const tf::TransformListener& tf_listener, const std::string& frame, const std::string& in_frame, geometry_msgs::PoseStamped& stamped_pose, const double& timeout = 10.0);
bool getLatestFrameInFrame( const tf::TransformListener& tf_listener, const std::string& frame, const std::string& in_frame, geometry_msgs::PoseStamped& stamped_pose, const double& max_age = 2.0);

/**
 * Adds XYZ in a particular frame.
 * This function converts a point to a frame, adds XYZ and then converts it back to its original frame
 * @param  tf      Transformlistener
 * @param  frame   In which frame the addition should be done
 * @param  x       x-value to add
 * @param  y       y-value to add
 * @param  z       z-value to add
 * @param  point   Reference to point where the addition is applied to
 * @param  timeout How long to wait for the transform to be available
 * @return         It the addition was succesful
 */
bool addXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PointStamped& point, const double timeout = 10.0);

/**
 * Adds XYZ in a particular frame.
 * This function converts a pose to a frame, adds XYZ and then converts it back to its original frame
 * @param  tf      Transformlistener
 * @param  frame   In which frame the addition should be done
 * @param  x       x-value to add
 * @param  y       y-value to add
 * @param  z       z-value to add
 * @param  pose    Reference to pose where the addition is applied to
 * @param  timeout How long to wait for the transform to be available
 * @return         It the addition was succesful
 */
bool addXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PoseStamped& pose, const double timeout = 10.0);

/**
 * Adds XYZ in a particular frame.
 * This function converts a point to a frame, adds XYZ and then converts it back to its original frame
 * @param  tf      Transformlistener
 * @param  frame   In which frame the addition should be done
 * @param  x       x-value to add
 * @param  y       y-value to add
 * @param  z       z-value to add
 * @param  point   Reference to point where the addition is applied to
 * @param  timeout How long to wait for the transform to be available
 * @return         It the addition was succesful
 */
bool addXYZInFrameNow ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PointStamped& point, const double timeout = 10.0);

/**
 * Adds XYZ in a particular frame.
 * This function converts a pose to a frame, adds XYZ and then converts it back to its original frame
 * @param  tf      Transformlistener
 * @param  frame   In which frame the addition should be done
 * @param  x       x-value to add
 * @param  y       y-value to add
 * @param  z       z-value to add
 * @param  pose    Reference to pose where the addition is applied to
 * @param  timeout How long to wait for the transform to be available
 * @return         It the addition was succesful
 */
bool addXYZInFrameNow ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PoseStamped& pose, const double timeout = 10.0);

/**
 * Sets XYZ in a particular frame.
 * This function converts a point to a frame, sets XYZ and then converts it back to its original frame
 * @param  tf      Transformlistener
 * @param  frame   In which frame the values should be set
 * @param  x       x-value to set
 * @param  y       y-value to set
 * @param  z       z-value to set
 * @param  point   Reference to point 
 * @param  timeout How long to wait for the transform to be available
 * @return         It the fucntion was succesful
 */
bool setXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PointStamped& point, const double timeout = 10.0);

/**
 * Sets XYZ in a particular frame.
 * This function converts a pose to a frame, sets XYZ and then converts it back to its original frame
 * @param  tf      Transformlistener
 * @param  frame   In which frame the values should be set
 * @param  x       x-value to set
 * @param  y       y-value to set
 * @param  z       z-value to set
 * @param  pose    Reference to pose 
 * @param  timeout How long to wait for the transform to be available
 * @return         It the fucntion was succesful
 */
bool setXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PoseStamped& pose, const double timeout = 10.0);

} // rose_transformations

#endif // TRANSFORMATIONS_HPP