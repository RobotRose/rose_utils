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
#include "rose_transformations/transformations.hpp"

namespace rose_transformations { 

bool transformToFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PointStamped& point, const double timeout )
{
    geometry_msgs::PointStamped    transformed_point;
    try
    {
        tf.waitForTransform(frame, point.header.frame_id, point.header.stamp, ros::Duration(timeout) );
        tf.transformPoint(frame, point, transformed_point);

        point = transformed_point;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PoseStamped& pose, const double timeout )
{
    geometry_msgs::PoseStamped    transformed_pose;
    try
    {
        tf.waitForTransform(frame, pose.header.frame_id, pose.header.stamp, ros::Duration(timeout) );
        tf.transformPose(frame, pose, transformed_pose);

        pose = transformed_pose;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToFrame( const tf::TransformListener& tf, const std::string frame, sensor_msgs::PointCloud& point_cloud, const double timeout )
{
    sensor_msgs::PointCloud    transformed_point_cloud;
    try
    {
        tf.waitForTransform(frame, point_cloud.header.frame_id, point_cloud.header.stamp, ros::Duration(timeout) );
        tf.transformPointCloud(frame, point_cloud, transformed_point_cloud);

        point_cloud = transformed_point_cloud;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PointStamped& point, const double timeout )
{
    geometry_msgs::PointStamped    transformed_point;
    try
    {
        tf.waitForTransform(frame, point.header.frame_id, ros::Time(0), ros::Duration(timeout) );
        tf.transformPoint(frame, point, transformed_point);

        point = transformed_point;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PoseStamped& pose, const double timeout )
{
    geometry_msgs::PoseStamped    transformed_pose;
    try
    {
        tf.waitForTransform(frame, pose.header.frame_id, ros::Time(0), ros::Duration(timeout) );
        tf.transformPose(frame, pose, transformed_pose);

        pose = transformed_pose;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, sensor_msgs::PointCloud& point_cloud, const double timeout )
{
    sensor_msgs::PointCloud transformed_point_cloud;
    try
    {
        tf.waitForTransform(frame, point_cloud.header.frame_id, ros::Time(0), ros::Duration(timeout) );
        tf.transformPointCloud(frame, point_cloud, transformed_point_cloud);

        point_cloud = transformed_point_cloud;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToLatestFrame( const tf::TransformListener& tf, const std::string frame, const rose_geometry::Stamped<rose_geometry::Point>& stamped_point, const double timeout )
{
    geometry_msgs::PointStamped point_msg;
    point_msg.header    = stamped_point.header;
    point_msg.point     = stamped_point.data.getROSmsg();

    return transformToLatestFrame(tf, frame, point_msg, timeout);
}

bool transformToFrameNow( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PointStamped& point, const double timeout )
{
    geometry_msgs::PointStamped    transformed_point;
    try
    {
        tf.waitForTransform(frame, point.header.frame_id, ros::Time::now(), ros::Duration(timeout) );
        tf.transformPoint(frame, point, transformed_point);

        point = transformed_point;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToFrameNow( const tf::TransformListener& tf, const std::string frame, geometry_msgs::PoseStamped& pose, const double timeout )
{
    geometry_msgs::PoseStamped    transformed_pose;
    try
    {
        tf.waitForTransform(frame, pose.header.frame_id, pose.header.stamp /*ros::Time::now()*/, ros::Duration(timeout) );
        tf.transformPose(frame, pose, transformed_pose);

        pose = transformed_pose;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool transformToFrameNow( const tf::TransformListener& tf, const std::string frame, sensor_msgs::PointCloud& point_cloud, const double timeout )
{
    sensor_msgs::PointCloud    transformed_point_cloud;
    try
    {
        tf.waitForTransform(frame, point_cloud.header.frame_id, ros::Time::now(), ros::Duration(timeout) );
        tf.transformPointCloud(frame, point_cloud, transformed_point_cloud);

        point_cloud = transformed_point_cloud;

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool getFrameInFrame( const tf::TransformListener& tf_listener, const std::string& frame, const std::string& in_frame, geometry_msgs::PoseStamped& stamped_pose, const double& timeout)
{
    tf::StampedTransform transform;
    try
    {
        tf_listener.waitForTransform(in_frame, frame, ros::Time::now(), ros::Duration(timeout) );
        tf_listener.lookupTransform(in_frame, frame, ros::Time(0), transform);

        stamped_pose.header.frame_id    = in_frame;
        stamped_pose.header.stamp       = transform.stamp_;
        stamped_pose.pose.position.x    = (double)transform.getOrigin().x();
        stamped_pose.pose.position.y    = transform.getOrigin().y();
        stamped_pose.pose.position.z    = transform.getOrigin().z();
        tf::quaternionTFToMsg (transform.getRotation(), stamped_pose.pose.orientation);

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool getLatestFrameInFrame( const tf::TransformListener& tf_listener, const std::string& frame, const std::string& in_frame, geometry_msgs::PoseStamped& stamped_pose, const double& max_age)
{
    tf::StampedTransform transform;
    try
    {
        // tf_listener.waitForTransform(in_frame, frame, ros::Time::now(), ros::Duration(timeout) );
        tf_listener.lookupTransform(in_frame, frame, ros::Time(0), transform);

        // Check if transform is not too old
        if((ros::Time::now() - transform.stamp_).toSec() > max_age)
        {
            ROS_WARN("Warning in getLatestFrameInFrame, age of transform: %.6f exceeds max age: %.6f.", (ros::Time::now() - transform.stamp_).toSec(), max_age);
            return false; 
        }

        stamped_pose.header.frame_id    = in_frame;
        stamped_pose.header.stamp       = ros::Time::now();
        stamped_pose.pose.position.x    = (double)transform.getOrigin().x();
        stamped_pose.pose.position.y    = transform.getOrigin().y();
        stamped_pose.pose.position.z    = transform.getOrigin().z();
        tf::quaternionTFToMsg (transform.getRotation(), stamped_pose.pose.orientation);

        return true;
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR("Received an exception: %s", ex.what());
    }

    return false;
}

bool addXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PointStamped& point, const double timeout)
{
    std::string old_frame = point.header.frame_id;
    // Transform to frame
    if ( not rose_transformations::transformToFrame(tf, frame, point, timeout) )
      return false ;

    // Add x,y,z
    point.point.x += x;
    point.point.y += y;
    point.point.z += z;

    // Transform back
    if ( not rose_transformations::transformToFrame(tf, old_frame, point, timeout) )
      return false;

    return true;
}

bool addXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PoseStamped& pose, const double timeout)
{
    std::string old_frame = pose.header.frame_id;

    // Transform to frame
    if ( not rose_transformations::transformToFrame(tf, frame, pose, timeout) )
      return false;

    // Add x,y,z
    pose.pose.position.x += x;
    pose.pose.position.y += y;
    pose.pose.position.z += z;

    // Transform back
    if ( not rose_transformations::transformToFrame(tf, old_frame, pose, timeout) )
      return false;

    return true;
}

bool addXYZInFrameNow ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PointStamped& point, const double timeout)
{
    std::string old_frame = point.header.frame_id;
    // Transform to frame
    if ( not rose_transformations::transformToFrameNow(tf, frame, point, timeout) )
      return false ;

    // Add x,y,z
    point.point.x += x;
    point.point.y += y;
    point.point.z += z;

    // Transform back
    if ( not rose_transformations::transformToFrameNow(tf, old_frame, point, timeout) )
      return false;

    return true;
}

bool addXYZInFrameNow ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PoseStamped& pose, const double timeout)
{
    std::string old_frame = pose.header.frame_id;

    // Transform to frame
    if ( not rose_transformations::transformToFrameNow(tf, frame, pose, timeout) )
      return false;

    // Add x,y,z
    pose.pose.position.x += x;
    pose.pose.position.y += y;
    pose.pose.position.z += z;

    // Transform back
    if ( not rose_transformations::transformToFrameNow(tf, old_frame, pose, timeout) )
      return false;

    return true;
}

bool setXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PointStamped& point, const double timeout)
{
    std::string old_frame = point.header.frame_id;

    // Transform to frame
    if ( not rose_transformations::transformToFrame(tf, frame, point, timeout));
      return false;

    // Add x,y,z
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;

    // Transform back
    if ( not rose_transformations::transformToFrame(tf, old_frame, point, timeout))
      return false;

    return true;
}

bool setXYZInFrame ( const tf::TransformListener& tf, const std::string frame, const double x, const double y, const double z, geometry_msgs::PoseStamped& pose, const double timeout)
{
    std::string old_frame = pose.header.frame_id;

    // Transform to frame
    if ( not rose_transformations::transformToFrame(tf, frame, pose, timeout) )
      return false;

    // Add x,y,z
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    // Transform back
    if ( not rose_transformations::transformToFrame(tf, old_frame, pose, timeout) )
      return false;

    return true;
}

} //namespace rose_transformations