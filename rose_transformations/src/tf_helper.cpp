/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/16
* 		- File created.
*
* Description:
*	A helper class that has to be used by all nodes publishing transforms.
* 
***********************************************************************************/
#include "tf_helper/tf_helper.hpp"

using std::string;

TFHelper::TFHelper(string transform_name, ros::NodeHandle n, string from_link, string to_link)
	: n_(n)
	, from_link_(from_link)
	, to_link_(to_link)
	, transform_name_(transform_name)
	, tf_(NULL)
{

}

TFHelper::~TFHelper()
{
	if(tf_ != NULL)
		delete tf_;
}

bool TFHelper::setTransform(float tf_roll, float tf_pitch, float tf_yaw, float tf_x, float tf_y, float tf_z)
{
	tf::Quaternion quat;
	quat.setRPY(tf_roll, tf_pitch, tf_yaw);
	return setTransform(quat, tf::Vector3(tf_x, tf_y, tf_z));
}

bool TFHelper::setTransform(tf::Quaternion quat, tf::Vector3 vect)
{
	tf::Transform transform = tf::Transform(quat, vect);
	return setTransform(transform);
}

bool TFHelper::setTransform(tf::Transform tf)
{
	// ROS_DEBUG_NAMED(ROS_NAME_TF, "Setting transform");
	if(tf_ == NULL)
		tf_ = new tf::Transform();

	*tf_ = tf;
	return true;
}

bool TFHelper::setTransform(const geometry_msgs::Pose& pose)
{
	geometry_msgs::Vector3 rpy = rose20_common::quaternionToRPY(pose.orientation);
	return setTransform(rpy.x, rpy.y, rpy.z, pose.position.x, pose.position.y, pose.position.z);
}

bool TFHelper::setTransform(const geometry_msgs::PoseStamped& pose_stamped)
{
	return setTransform(pose_stamped.pose);
}

bool TFHelper::setTransform(const geometry_msgs::PoseWithCovariance& pose_cov)
{
	return setTransform(pose_cov.pose);
}

bool TFHelper::setTransform(const geometry_msgs::PoseWithCovarianceStamped& pose_cov_stamped)
{
	return setTransform(pose_cov_stamped.pose.pose);
}

// Could return NULL
tf::Transform* TFHelper::getTransform() const
{
	return tf_;
}

bool TFHelper::Broadcast() //! @todo: OH rename to broadcast()
{
	if(tf_ != NULL)
	{
		// ROS_DEBUG_NAMED(ROS_NAME_TF, "Broadcasting transform %s -> %s", from_link_.c_str(), to_link_.c_str());
		tf_broadcaster_.sendTransform(tf::StampedTransform(*tf_, ros::Time::now(), from_link_, to_link_));
	}
	else
		ROS_WARN_THROTTLE_NAMED(1, ROS_NAME_TF, "Transform was not set when trying to broadcast.");

	return true;
}

string TFHelper::getName() const
{
	return transform_name_;
}

void TFHelper::setName(string new_name)
{
	transform_name_ = new_name;
}
