/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
* Author: Okke Hendriks
* Date  : 2013/12/16
*     - File created.
*
* Description:
* A helper class that has to be used by all nodes publishing transforms.
* 
***********************************************************************************/
#ifndef TF_HELPER_HPP
#define TF_HELPER_HPP

#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "rose_common/common.hpp"

#define ROS_NAME_TF     (ROS_NAME + "|TF")

using std::string;

class TFHelper
{
  public:
    TFHelper(string transform_name, ros::NodeHandle n, string from_link, string to_link);
    ~TFHelper();

    bool setTransform(float tf_roll, float tf_pitch, float tf_yaw, float tf_x, float tf_y, float tf_z);
    bool setTransform(tf::Quaternion quat, tf::Vector3 vect);
    bool setTransform(tf::Transform tf);
    bool setTransform(const geometry_msgs::Pose& pose);
    bool setTransform(const geometry_msgs::PoseStamped& pose_stamped);
    bool setTransform(const geometry_msgs::PoseWithCovariance& pose_cov);
    bool setTransform(const geometry_msgs::PoseWithCovarianceStamped& pose_cov_stamped);

    tf::Transform*  getTransform() const;
    bool            Broadcast();
    string          getName() const;
    void            setName(string new_name);

  protected:
    ros::NodeHandle             n_;
    tf::TransformBroadcaster    tf_broadcaster_;
    tf::Transform*              tf_;
    string                      transform_name_;
    string                      from_link_;
    string                      to_link_;
};

#endif  // TF_HELPER_HPP