/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*  Author: Mathijs de Langen
*  Date  : 2014/02/27
*     - File created.
*
* Description:
*  description
* 
***********************************************************************************/
#ifndef BOUNDING_BOX_FACTORY_HPP
#define BOUNDING_BOX_FACTORY_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>

#include "bounding_box/bounding_box.hpp"
#include "rose_common/common.hpp"


class BoundingBoxFactory
{
  public:
    BoundingBoxFactory();
    ~BoundingBoxFactory();

    BoundingBox createBoundingBox( std::string text );
    BoundingBox createBoundingBox( const sensor_msgs::PointCloud2& point_cloud, int x1, int y1, int x2, int y2 );

  private:
  	tf::TransformListener 		tf_;

  	ros::Publisher   point_cloud_pub_;
    ros::Publisher   marker_pub_;
  	ros::NodeHandle  n;
    visualization_msgs::Marker marker_;

};

#endif //BOUNDING_BOX_FACTORY_HPP