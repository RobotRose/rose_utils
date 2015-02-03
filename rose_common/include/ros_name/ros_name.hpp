#ifndef ROS_NAME_HPP
#define ROS_NAME_HPP

#include <boost/algorithm/string.hpp>

//! define a shorthand for getting the name the current node trimming any leading /'s (before the smc include because it uses this define)
#define ROS_NAME 	(boost::algorithm::trim_left_copy_if(ros::this_node::getName(), boost::algorithm::is_any_of("/")))

#endif // ROS_NAME_HPP
