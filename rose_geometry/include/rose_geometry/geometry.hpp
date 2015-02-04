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

namespace rose_geometry{

//! @todo OH: Move to a vector class 
//! @todo OH: Also add a point class
void    rotateVect(float* x, float* y, float radians);
void    rotateVect(double* x, double* y, double radians);
float   getShortestSignedAngle(float v1_x, float v1_y, float v2_x, float v2_y);
float   getShortestSignedAngle(float v1_x, float v1_y, float radians2);
float   getShortestSignedAngle(float radians1, float radians2);
float   getShortestSignedAngle(const Point& p1 , const Point& p2);
void    setVectorLengthXY(float* x, float* y, float wanted_length);
void    setVectorLengthXY(double* x, double* y, double wanted_length);
void    limitVectorLengthXY(float* x, float* y, float limit_length);
void    limitVectorLengthXY(double* x, double* y, double limit_length);
float   getAngle(const Point& point_a, const Point& point_b);
float   getAngle(const float& angle, const Point& point_b);
float   getAngle(const Pose& pose_a, const Pose& pose_b);

bool limit(float min, float max, float* number);
bool limit(double min, double max, double* number);
bool limit(int min, int max, int* number);

bool  wrapToHalfPi(float* angle);
float wrapToHalfPi(float angle);
float wrapToPi(float angle);

float distanceXY(const Point& point_a, const Point& point_b);
float distanceXY(const Pose& pose_a, const Pose& pose_b);
float distanceXY(const PoseStamped& stamped_pose_a, const PoseStamped& stamped_pose_b);

float distanceXYZ(const Point& point_a, const Point& point_b);
float distanceXYZ(const Pose& pose_a, const Pose& pose_b);
float distanceXYZ(const PoseStamped& stamped_pose_a, const PoseStamped& stamped_pose_b);

float distanceXYsquared(const Point& point_a, const Point& point_b);
float distanceXYsquared(const Pose& pose_a, const Pose& pose_b);
float distanceXYsquared(const PoseStamped& stamped_pose_a, const PoseStamped& stamped_pose_b);

float distanceXYZsquared(const Point& point_a, const Point& point_b);
float distanceXYZsquared(const Pose& pose_a, const Pose& pose_b);
float distanceXYZsquared(const PoseStamped& stamped_pose_a, const PoseStamped& stamped_pose_b);

bool pointInsideOrOnCircle(const Point& point, const Point& circle_center, const float& radius);
bool isPointInPolygon(const rose20_common::geometry::Point& point, const vector<rose20_common::geometry::Point>& polygon_points);
vector<rose20_common::geometry::Point> intersectionsLineSegmentLineSegment(     const rose20_common::geometry::Point& l1a, 
                                                                                const rose20_common::geometry::Point& l1b, 
                                                                                const rose20_common::geometry::Point& l2a, 
                                                                                const rose20_common::geometry::Point& l2b);

vector<rose20_common::geometry::Point> intersectionsLineSegmentCircle(  const rose20_common::geometry::Point& linepoint_a, 
                                                                        const rose20_common::geometry::Point& linepoint_b, 
                                                                        const rose20_common::geometry::Point& circle_center, 
                                                                        const float& r);

float distancePointLine(const Point& p, const Point& linepoint_a, const Point& linepoint_b);
float distancePointLine(    const rose20_common::geometry::Point& p, 
                            const rose20_common::geometry::Point& linepoint_a, 
                            const rose20_common::geometry::Point& linepoint_b);
float minimalDistancePointPolygon( const rose20_common::geometry::Point& p, const vector<rose20_common::geometry::Point>& poly_points);
float dot(const Point& a, const Point& b);
float cross(const Point& a, const Point& b);
bool aligned(const rose20_common::geometry::Point& a, const rose20_common::geometry::Point&b, const rose20_common::geometry::Point& c);
bool isBetween(const rose20_common::geometry::Point& a, const rose20_common::geometry::Point&b, const rose20_common::geometry::Point& c);
bool isBetweenOrEqual(const float& a, const float&b, const float& c);

geometry_msgs::PoseStamped translatePose(const geometry_msgs::PoseStamped& stamped_pose, const geometry_msgs::Vector3& translation);
geometry_msgs::PoseStamped rotatePose(const geometry_msgs::PoseStamped& stamped_pose, const tf::Quaternion& quat_tf_rotation);
geometry_msgs::PoseStamped rotatePose(const geometry_msgs::PoseStamped& stamped_pose, const geometry_msgs::Quaternion& quat);
void rotatePointAroundOrigin(Point& point, float radians);
void rotatePointAroundOrigin(rose20_common::geometry::Point& point, float radians);
void rotatePointsAroundOrigin(float radians, std::vector<Point>& points);
void rotatePointsAroundOrigin(float radians, std::vector<rose20_common::geometry::Point>& points);
geometry_msgs::Vector3 rotate2DVector(const geometry_msgs::Vector3& vect, float radians);
void translatePoint(const float& dx, const float& dy, Point& point);
void translatePoint(const float& dx, const float& dy, rose20_common::geometry::Point& point);
void translatePoints(const float& dx, const float& dy, std::vector<Point>& points);
void translatePoints(const float& dx, const float& dy, std::vector<rose20_common::geometry::Point>& points);
bool areClockwise(const Point& v1, const Point& v2);

float   getVectorLengthXY(float x, float y);
double  getVectorLengthXY(double x, double y);
float   getVectorLengthXYZ(float x, float y, float z);
double  getVectorLengthXYZ(double x, double y, double z);

template <typename T> 
int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}

} // namespace rose_geometry

#endif // GEOMETRY_HPP