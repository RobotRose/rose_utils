/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/02/27
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "bounding_box_factory/bounding_box_factory.hpp"

BoundingBoxFactory::BoundingBoxFactory()
{
	ROS_INFO("BoundingBoxFactory::BoundingBoxFactory()");
	point_cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("/bounding_box_factory/cloud",10, true);
	marker_pub_      = n.advertise<visualization_msgs::Marker> ("/bounding_box_factory/marker", 1);
}

BoundingBoxFactory::~BoundingBoxFactory()
{

}

BoundingBox BoundingBoxFactory::createBoundingBox( std::string text )
{
	geometry_msgs::Point center;
	geometry_msgs::Quaternion orientation;

	size_t count = std::count(text.begin(), text.end(), ',');
	if (count != 2)
	{
		ROS_ERROR("Inserted bounding box is not correct");
		BoundingBox bb;
		return bb;
	}

	center.x = rose_conversions::stringToDouble(text.substr(0, text.find_first_of(",")));
	text.erase(0, text.find_first_of(",")+1);
	center.y = rose_conversions::stringToDouble(text.substr(0, text.find_first_of(",")));
	text.erase(0, text.find_first_of(",")+1);
	center.z = rose_conversions::stringToDouble(text);

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = "/map";
	pose_stamped.header.stamp 	 = ros::Time::now();
	pose_stamped.header.seq 	 = 0;

	BoundingBox bb(pose_stamped, 0,0,0);

	return bb;
}

BoundingBox BoundingBoxFactory::createBoundingBox( const sensor_msgs::PointCloud2& point_cloud, int x1, int y1, int x2, int y2 )
{
	geometry_msgs::Point easy_center;

	ROS_INFO("BoundingBoxFactory::createBoundingBox");
	geometry_msgs::Point center;
	geometry_msgs::Quaternion orientation;
	float obj_height = 0;
	float obj_width = 0;

	// Copied (part of the) ugly code of rect_marker.cpp
	//! @todo MdL: clean up / remove
	int cloud_width = point_cloud.width;
	int cloud_height = point_cloud.height;	
	pcl::PointCloud<pcl::PointXYZ> cloudPt;
	pcl::fromROSMsg (point_cloud, cloudPt);
	
	float lx,ly,lz, rx, ry, rz;
	float wlx,wly,wlz, wrx, wry, wrz;	

	pcl::PointCloud<pcl::PointXYZ> object_cloud;

	if(x2 <= x1 || y2 <= y1) 
		ROS_INFO("Invalid rect");
	else
	{
		ROS_INFO("Valid rect x1: %d, x2: %d, y1: %d, y2: %d", x1, x2, y1, y2);
		int width = x2 - x1; 
		int height = y2 - y1;

		object_cloud.width = width;
		object_cloud.height = height;
		object_cloud.is_dense = false;
		object_cloud.points.resize (object_cloud.width * object_cloud.height);

		float minX, maxX, minY, maxY, x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, x_4, y_4, z_4 = 0.0;
		geometry_msgs::Point sum_of_points;
		int counter = 0;

		sum_of_points.x += 0.0;
		sum_of_points.y += 0.0;
		sum_of_points.z += 0.0;


		// Mathijs implementation
		for ( int j =  0; j < height ; j++)
		{
			for ( int i = 0 ; i < width ; i++ )
			{
				// ROS_INFO("point %d/%d",j*width, width*height);
				object_cloud.points[i+j*width].x = cloudPt.points[(i+x1)+cloud_width*(j+y1)].x;
				object_cloud.points[i+j*width].y = cloudPt.points[(i+x1)+cloud_width*(j+y1)].y;
				object_cloud.points[i+j*width].z = cloudPt.points[(i+x1)+cloud_width*(j+y1)].z;
			
				if (object_cloud.points[width*i+j].z > 0) // not nan
				{
					sum_of_points.x += object_cloud.points[width*i+j].x;
					sum_of_points.y += object_cloud.points[width*i+j].y;
					sum_of_points.z += object_cloud.points[width*i+j].z;
					counter++;
				}
			}
		}

		sum_of_points.x = sum_of_points.x / counter;
		sum_of_points.y = sum_of_points.y / counter;
		sum_of_points.z = sum_of_points.z / counter;

		ROS_INFO ( "average of points x: %f,y: %f ,z: %f", sum_of_points.x, sum_of_points.y, sum_of_points.z);

		// pcl::io::savePCDFileASCII ("test_pcd.pcd", object_cloud);
		// ROS_INFO ("Saved %d data points to test_pcd.pcd.", (int)object_cloud.points.size ());

		int centerx = (width/2.0);
		int centery = (height/2.0);
		int nr_points_found = 0;
		int offset = width * height;
		int index_nr = -1;
		bool left = true;
		bool right = true;
		
		for(int lindex = 1; lindex < width/2.0; lindex++)
		{
			int ind = width * (height / 2.0) +  width / 0.2;
			if(left && object_cloud.points[ind - lindex].z > 0.15)// && object_cloud.points[ind - lindex].z < 2.0 )
			{
				x_1 = object_cloud.points[ind - lindex].x;
				y_1 = object_cloud.points[ind - lindex].y;
				z_1 = object_cloud.points[ind - lindex].z;
				ROS_INFO("Left x: %f, y: %f, z: %f", x_1, y_1, z_1);
				left = false;
				if ( lindex < offset )
				{
					index_nr = ind - lindex;
					offset = lindex;
					ROS_INFO("offset changed: %d", offset);
				}
			}

			if(right && object_cloud.points[ind + lindex].z > 0.15)// && object_cloud.points[ind + lindex].z < 2.0)
			{
				x_2 = object_cloud.points[ind + lindex].x;
				y_2 = object_cloud.points[ind + lindex].y;
				z_2 = object_cloud.points[ind + lindex].z;
				ROS_INFO("Right x: %f, y: %f, z: %f", x_2, y_2, z_2);
				right = false;
				nr_points_found++;
				if ( lindex < offset )
				{
					index_nr = ind + lindex;		
					offset = lindex;		
					ROS_INFO("offset changed: %d", offset);
				}
			}
		}

		bool up = true;
		bool down = true;
		for(int lindex = 1; lindex < height/2.0; lindex++)
		{
			int ind = height * width / 2.0 + height / 2.0;
			if(down && object_cloud.points[ind - lindex*width].z > 0.15)// && object_cloud.points[ind - lindex].z < 2.0 )
			{
				x_3 = object_cloud.points[ind - lindex*width].x;
				y_3 = object_cloud.points[ind - lindex*width].y;
				z_3 = object_cloud.points[ind - lindex*width].z;
				ROS_INFO("Down x: %f, y: %f, z: %f", x_3, y_3, z_3);
				down = false;
				if ( lindex < offset )
				{
					index_nr = ind - lindex*width;
					offset = lindex;
					ROS_INFO("offset changed: %d", offset);
				}
			}

			if(up && object_cloud.points[ind + lindex*width].z > 0.15)// && object_cloud.points[ind + lindex].z < 2.0)
			{
				x_4 = object_cloud.points[ind + lindex*width].x;
				y_4 = object_cloud.points[ind + lindex*width].y;
				z_4 = object_cloud.points[ind + lindex*width].z;
				ROS_INFO("Up x: %f, y: %f, z: %f", x_4, y_4, z_4);
				up = false;
				if ( lindex < offset )
				{
					index_nr = ind + lindex*width;
					offset = lindex;
					ROS_INFO("offset changed: %d", offset);
				}
			}
		}

		index_nr = (height/2) * width +  (width-1);

		// easy_center.x = sum_of_points.x;
		// easy_center.y = sum_of_points.y;
		// easy_center.z = sum_of_points.z;
		
		easy_center.x = object_cloud.points[index_nr].x;
		easy_center.y = object_cloud.points[index_nr].y;
		easy_center.z = object_cloud.points[index_nr].z;
		
		if ( not easy_center.x > 0 || not easy_center.y > 0 || not  easy_center.z > 0 )
		{
			ROS_ERROR("Invalid bounding box");
			geometry_msgs::PoseStamped invalid_pose;
			BoundingBox bb_invalid = BoundingBox(invalid_pose,0.0,0.0,0.0);
			return bb_invalid;
		}
		
		ROS_INFO("center point: (%f,%f,%f)",easy_center.x,easy_center.y,easy_center.z);


		// ////////////////////////////// ADDING /////////////////////////////
		// ROS_INFO("--------------UPPER POINT DETECTION---------");
		// bool uleft = true;
		// bool bright = true;
		// for(int lindex = 1; lindex < width/4.0; lindex++)
		// {
		// 	int ind = (width/2.0);
		// 	if(uleft && object_cloud.points[ind + lindex].z > 0.15)
		// 	{
		// 		lx = object_cloud.points[ind + lindex].x;
		// 		ly = object_cloud.points[ind + lindex].y;
		// 		lz = object_cloud.points[ind + lindex].z;
		// 		ROS_INFO("good point @ top");
		// 		uleft = false;
		// 	}
		// }
		// ROS_INFO("--------------LOWER POINT DETECTION---------");

		// for(int lindex = 1; lindex < width/4.0; lindex++)
		// {
		// 	int ind = (width * (height-1)) + (width/2.0);

		// 	if(bright && object_cloud.points[ind - lindex].z > 0.15)
		// 	{
		// 		rx = object_cloud.points[ind - lindex].x;
		// 		ry = object_cloud.points[ind - lindex].y;
		// 		rz = object_cloud.points[ind - lindex].z;
		// 		ROS_INFO("good point @ bottom");
		// 		bright = false;
		// 	}

		// }
		// ROS_INFO("--------------LEFT POINT DETECTION---------");
		// bool wleft = true;
		// bool wright = true;
		// for(int lindex = 1; lindex < width/4.0; lindex++)
		// {
		// 	int ind = (width*(height/2.0)) + 1;
		// 	if(wleft && object_cloud.points[ind + lindex].z > 0.15)
		// 	{
		// 		wlx = object_cloud.points[ind + lindex].x;
		// 		wly = object_cloud.points[ind + lindex].y;
		// 		wlz = object_cloud.points[ind + lindex].z;
		// 		ROS_INFO("good point @ left");
		// 		wleft = false;
		// 	}
		// }
		// ROS_INFO("--------------RIGHT POINT DETECTION---------");

		// for(int lindex = 1; lindex < width/4.0; lindex++)
		// {
		// 	int ind = (width*(height/2.0)) + (width - 1);

		// 	if(wright && object_cloud.points[ind - lindex].z > 0.15)
		// 	{
		// 		wrx = object_cloud.points[ind - lindex].x;
		// 		wry = object_cloud.points[ind - lindex].y;
		// 		wrz = object_cloud.points[ind - lindex].z;
		// 		ROS_INFO("good point @ right");
		// 		wright = false;
		// 	}

		// }
		// ///////////////////////////////////////////////////////////////////

		// // Added IF construct remove for old state
		// if(left || right || uleft || bright || wleft || wright)
		// {		
		//     ROS_ERROR("tehre is a bad point");
		// }
		// else
		// {
		// 	std::stringstream mode_str;
		// 	mode_str<<"";
		// 	ROS_INFO("Told user the point is good");

		// 	ROS_INFO("MIN_X: %f, MAX_X: %f, MIN_Y: %f, MAX_Y: %f", minX,maxX,minY,maxY);
		// 	ROS_INFO("left points (%f, %f, %f)", x_1, y_1, z_1);
		// 	ROS_INFO("right points (%f, %f, %f)", x_2, y_2, z_2);
		// 	//ROS_INFO("upper points (%f, %f, %f)", x_3, y_3, z_3);
		// 	//ROS_INFO("lower points (%f, %f, %f)", x_4, y_4, z_4);
		// 	ROS_INFO("AVERAGE DIST: %f",(z_1 + z_2)/2.0);
		// 	ROS_INFO("New point: (%f, %f, %f)",(x_1 + x_2)/2.0, (y_1 + y_2)/2.0,(z_1 + z_2)/2.0);


		// 	ROS_INFO("upper corner points (%f, %f, %f)", lx,ly,lz);
		// 	ROS_INFO("lower corner points (%f, %f, %f)", rx,ry,rz);
		// 	ROS_INFO("left corner points (%f, %f, %f)", wlx,wly,wlz);
		// 	ROS_INFO("right corner points (%f, %f, %f)", wrx,wry,wrz);

		// 	obj_height = sqrt(pow(lx - rx,2.0) + pow(ly - ry,2.0) + pow(lz - rz,2.0));
		// 	ROS_INFO("Object Height: %f",obj_height);
		// 	obj_width = sqrt(pow(wlx - wrx,2.0) + pow(wly - wry,2.0) + pow(wlz - wrz,2.0));
		// 	ROS_INFO("Object width: %f",obj_width);

		// 	center.x = (x_1 + x_2)/2.0;
		// 	center.y = (y_1 + y_2)/2.0;
		// 	center.z = (z_1 + z_2)/2.0;
		// }
	}

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(object_cloud, output);
	output.header.frame_id = "camera_rgb_optical_frame";

	point_cloud_pub_.publish(output);

	ROS_INFO("Bounding box factory: created bounding box");

	geometry_msgs::PoseStamped bounding_box_pose_stamped;
	
	bounding_box_pose_stamped.header 			= output.header;
	bounding_box_pose_stamped.pose.position 	= easy_center;
	bounding_box_pose_stamped.pose.orientation 	= orientation;

	BoundingBox bb(bounding_box_pose_stamped, obj_width, obj_height, 0);

  	visualization_msgs::Marker marker;
    
    marker.ns               = "bounding_box_factory";
    marker.id               = 0;
    marker.type             = visualization_msgs::Marker::CUBE;
    marker.action           = visualization_msgs::Marker::ADD;

    marker.header.frame_id  = bounding_box_pose_stamped.header.frame_id;
    marker.header.stamp     = ros::Time::now();

    marker.pose.position    = easy_center;
    
    marker.scale.x          = 0.02;
    marker.scale.y          = 0.02;
    marker.scale.z          = 0.02;
                  
	marker.color.r 			= 0.0f;
	marker.color.g 			= 1.0f;
	marker.color.b 			= 0.0f;
	marker.color.a 			= 1.0;

    marker.lifetime = ros::Duration(25);

	marker_pub_.publish(marker);

	return bb;
}