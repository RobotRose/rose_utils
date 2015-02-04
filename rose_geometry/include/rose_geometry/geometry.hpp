/***********************************************************************************
* Copyright: Rose B.V. (2015)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2015/02/03
* 		- File created.
*
* Description:
*	Geometry
* 
***********************************************************************************/
#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_listener.h> 

#include "rose_geometry/point.hpp"
#include "rose_geometry/stamped.hpp"

using std::vector;

namespace rose_geometry{

//! @todo OH: Move to a vector class 
//! @todo OH: Also add a point class
void    rotateVect(float* x, float* y, float radians);
void    rotateVect(double* x, double* y, double radians);
float   getShortestSignedAngle(float v1_x, float v1_y, float v2_x, float v2_y);
float   getShortestSignedAngle(float v1_x, float v1_y, float radians2);
float   getShortestSignedAngle(float radians1, float radians2);
float   getShortestSignedAngle(const geometry_msgs::Point& p1 , const geometry_msgs::Point& p2);
void    setVectorLengthXY(float* x, float* y, float wanted_length);
void    setVectorLengthXY(double* x, double* y, double wanted_length);
void    limitVectorLengthXY(float* x, float* y, float limit_length);
void    limitVectorLengthXY(double* x, double* y, double limit_length);
float   getAngle(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b);
float   getAngle(const float& angle, const geometry_msgs::Point& point_b);
float   getAngle(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b);

bool  wrapToHalfPi(float* angle);
float wrapToHalfPi(float angle);
float wrapToPi(float angle);

float distanceXY(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b);
float distanceXY(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b);
float distanceXY(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b);

float distanceXYZ(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b);
float distanceXYZ(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b);
float distanceXYZ(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b);

float distanceXYsquared(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b);
float distanceXYsquared(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b);
float distanceXYsquared(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b);

float distanceXYZsquared(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b);
float distanceXYZsquared(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b);
float distanceXYZsquared(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b);

bool pointInsideOrOnCircle(const geometry_msgs::Point& point, const geometry_msgs::Point& circle_center, const float& radius);
bool isPointInPolygon(const rose_geometry::Point& point, const vector<rose_geometry::Point>& polygon_points);
vector<rose_geometry::Point> intersectionsLineSegmentLineSegment(     const rose_geometry::Point& l1a, 
                                                                                const rose_geometry::Point& l1b, 
                                                                                const rose_geometry::Point& l2a, 
                                                                                const rose_geometry::Point& l2b);

vector<rose_geometry::Point> intersectionsLineSegmentCircle(  const rose_geometry::Point& linepoint_a, 
                                                                        const rose_geometry::Point& linepoint_b, 
                                                                        const rose_geometry::Point& circle_center, 
                                                                        const float& r);

float distancePointLine(const geometry_msgs::Point& p, const geometry_msgs::Point& linepoint_a, const geometry_msgs::Point& linepoint_b);
float distancePointLine(    const rose_geometry::Point& p, 
                            const rose_geometry::Point& linepoint_a, 
                            const rose_geometry::Point& linepoint_b);
float minimalDistancePointPolygon( const rose_geometry::Point& p, const vector<rose_geometry::Point>& poly_points);
float dot(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
float cross(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
bool aligned(const rose_geometry::Point& a, const rose_geometry::Point&b, const rose_geometry::Point& c);
bool isBetween(const rose_geometry::Point& a, const rose_geometry::Point&b, const rose_geometry::Point& c);
bool isBetweenOrEqual(const float& a, const float&b, const float& c);

geometry_msgs::PoseStamped translatePose(const geometry_msgs::PoseStamped& stamped_pose, const geometry_msgs::Vector3& translation);
geometry_msgs::PoseStamped rotatePose(const geometry_msgs::PoseStamped& stamped_pose, const tf::Quaternion& quat_tf_rotation);
geometry_msgs::PoseStamped rotatePose(const geometry_msgs::PoseStamped& stamped_pose, const geometry_msgs::Quaternion& quat);
void rotatePointAroundOrigin(geometry_msgs::Point& point, float radians);
void rotatePointAroundOrigin(rose_geometry::Point& point, float radians);
void rotatePointsAroundOrigin(float radians, std::vector<Point>& points);
void rotatePointsAroundOrigin(float radians, std::vector<rose_geometry::Point>& points);
geometry_msgs::Vector3 rotate2DVector(const geometry_msgs::Vector3& vect, float radians);
void translatePoint(const float& dx, const float& dy, geometry_msgs::Point& point);
void translatePoint(const float& dx, const float& dy, rose_geometry::Point& point);
void translatePoints(const float& dx, const float& dy, std::vector<Point>& points);
void translatePoints(const float& dx, const float& dy, std::vector<rose_geometry::Point>& points);
bool areClockwise(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2);

float   getVectorLengthXY(float x, float y);
double  getVectorLengthXY(double x, double y);
float   getVectorLengthXYZ(float x, float y, float z);
double  getVectorLengthXYZ(double x, double y, double z);

} // namespace rose_geometry

#endif // GEOMETRY_HPP