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
#include "rose_geometry/geometry.hpp"

namespace rose_geometry{

// Amount and direction you have to rotate v1 to end up at v2
float getShortestSignedAngle(float v1_x, float v1_y, float v2_x, float v2_y)
{ 
    setVectorLengthXY(&v1_x, &v1_y, 1.0);
    setVectorLengthXY(&v2_x, &v2_y, 1.0);
    
    float crossProduct = v1_x*v2_y - v1_y*v2_x;
    float dotProduct   = v1_x*v2_x + v1_y*v2_y;
    float theta        = atan2(crossProduct, dotProduct);
    return theta;
}

float getShortestSignedAngle(float v1_x, float v1_y, float radians2)
{ 
    float v2_x = 1.0;
    float v2_y = 0.0;
    rotateVect(&v2_x, &v2_y, radians2);
    return getShortestSignedAngle(v1_x, v1_y, v2_x, v2_y);
}

float getShortestSignedAngle(float radians1, float radians2)
{ 
    float v1_x = 1.0;
    float v1_y = 0.0;
    float v2_x = 1.0;
    float v2_y = 0.0;
    rotateVect(&v1_x, &v1_y, radians1);
    rotateVect(&v2_x, &v2_y, radians2);
    return getShortestSignedAngle(v1_x, v1_y, v2_x, v2_y);
}

float getShortestSignedAngle(const geometry_msgs::Point& p1 , const geometry_msgs::Point& p2)
{ 
    float v1_x = p1.x;
    float v1_y = p1.y;
    float v2_x = p2.x;
    float v2_y = p2.y;
    return getShortestSignedAngle(v1_x, v1_y, v2_x, v2_y);
}

// Rotate a vector given by x & y by radians
void rotateVect(float* x, float* y, float radians) //! @todo : rename to rotateVectorXY
{
  float x_temp = *x;
  float y_temp = *y;
  *x = x_temp*cos(radians) - y_temp*sin(radians);
  *y = y_temp*cos(radians) + x_temp*sin(radians);
}

// Rotate a vector given by x & y by radians
void rotateVect(double* x, double* y, double radians) //! @todo : rename to rotateVectorXY
{
  double x_temp = *x;
  double y_temp = *y;
  *x = x_temp*cos(radians) - y_temp*sin(radians);
  *y = y_temp*cos(radians) + x_temp*sin(radians);
}

float getVectorLengthXY(float x, float y)
{
  return sqrt(x*x + y*y);
}

double getVectorLengthXY(double x, double y)
{
  return sqrt(x*x + y*y);
}

float getVectorLengthXYZ(float x, float y, float z)
{
  return sqrt(x*x + y*y + z*z);
}

double getVectorLengthXYZ(double x, double y, double z)
{
  return sqrt(x*x + y*y + z*z);
}

// Set the length of a vector
void setVectorLengthXY(float* x, float* y, float wanted_length)
{
  float cur_length     = getVectorLengthXY(*x, *y);
  
  if(cur_length == 0.0)
    return;

  float factor         = wanted_length/cur_length;

  *x *= factor; 
  *y *= factor;
}

// Set the length of a vector
void setVectorLengthXY(double* x, double* y, double wanted_length)
{
  double cur_length     = getVectorLengthXY(*x, *y);
  
  if(cur_length == 0.0)
    return;

  double factor         = wanted_length/cur_length;

  *x *= factor; 
  *y *= factor;
}

// Limit the length of a vector
void limitVectorLengthXY(float* x, float* y, float limit_length)
{
  float cur_length     = getVectorLengthXY(*x, *y);
  
  if(cur_length == 0.0)
    return;

  if(cur_length <= limit_length)
    return;

  float factor          = limit_length/cur_length;

  *x *= factor; 
  *y *= factor;
}

// Limit the length of a vector
void limitVectorLengthXY(double* x, double* y, double limit_length)
{
  double cur_length     = getVectorLengthXY(*x, *y);
  
  if(cur_length == 0.0)
    return;

  if(cur_length <= limit_length)
    return;

  double factor          = limit_length/cur_length;

  *x *= factor; 
  *y *= factor;
}

float getAngle(const geometry_msgs::Point& point_a, const Point& point_b)
{
    float dx        = point_b.x - point_a.x;
    float dy        = point_b.y - point_a.y;
    return atan2(dy, dx);
}

float getAngle(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b)
{
    float dx        = pose_b.position.x - pose_a.position.x;
    float dy        = pose_b.position.y - pose_a.position.y;
    return atan2(dy, dx);
}

float getAngle(const float& angle, const Point& point_b)
{
    float v1_x = 1.0;
    float v1_y = 0.0;
    rotateVect(&v1_x, &v1_y, angle);

    float dx        = point_b.x - v1_x;
    float dy        = point_b.y - v1_y;

    return atan2(dy, dx);
}

//! @todo Wrap functions do now not work with angles outside range [-3*pi, 3*pi]
// Returns true if angle is altered
//! @todo: Make unit tests
bool wrapToHalfPi(float* angle)
{
    if(*angle >= M_PI/2.0)
    {
        *angle -= M_PI;
        return true;
    }
    else if(*angle <= -M_PI/2.0)
    {
        *angle += M_PI;
        return true;
    }

    return false;
}

// Returns angle wrapped to range -PI/2 -> PI/2
//! @todo: Make unit tests
inline float wrapToHalfPi(float angle)
{
    if (angle>0)
        angle = fmod(angle+0.5*M_PI, 2.0*M_PI)-0.5*M_PI;
    else
        angle = fmod(angle-0.5*M_PI, 2.0*M_PI)+0.5*M_PI;

    return angle;
}

// Returns angle wrapped to range -PI -> PI
inline float wrapToPi(float angle)
{
    if (angle>0)
        angle = fmod(angle+M_PI, 2.0*M_PI)-M_PI;
    else
        angle = fmod(angle-M_PI, 2.0*M_PI)+M_PI;

    return angle;
}

float distanceXY(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b)
{
    float dx = point_b.x - point_a.x;
    float dy = point_b.y - point_a.y;
    return sqrt(dx*dx + dy*dy);
}

float distanceXY(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b)
{
    return distanceXY(pose_a.position, pose_b.position);
}

float distanceXY(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b)
{
    return distanceXY(stamped_pose_a.pose.position, stamped_pose_b.pose.position);
}

float distanceXYZ(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b)
{
    float dx = point_b.x - point_a.x;
    float dy = point_b.y - point_a.y;
    float dz = point_b.z - point_a.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

float distanceXYZ(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b)
{
    return distanceXYZ(pose_a.position, pose_b.position);
}

float distanceXYZ(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b)
{
    return distanceXYZ(stamped_pose_a.pose.position, stamped_pose_b.pose.position);
}

float distanceXYsquared(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b)
{
    float dx = point_b.x - point_a.x;
    float dy = point_b.y - point_a.y;
    return (dx*dx + dy*dy);
}

float distanceXYsquared(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b)
{
    return distanceXYsquared(pose_a.position, pose_b.position);
}

float distanceXYsquared(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b)
{
    return distanceXYsquared(stamped_pose_a.pose.position, stamped_pose_b.pose.position);
}

float distanceXYZsquared(const geometry_msgs::Point& point_a, const geometry_msgs::Point& point_b)
{
    float dx = point_b.x - point_a.x;
    float dy = point_b.y - point_a.y;
    float dz = point_b.z - point_a.z;
    return (dx*dx + dy*dy + dz*dz);
}

float distanceXYZsquared(const geometry_msgs::Pose& pose_a, const geometry_msgs::Pose& pose_b)
{
    return distanceXYZsquared(pose_a.position, pose_b.position);
}

float distanceXYZsquared(const geometry_msgs::PoseStamped& stamped_pose_a, const geometry_msgs::PoseStamped& stamped_pose_b)
{
    return distanceXYZsquared(stamped_pose_a.pose.position, stamped_pose_b.pose.position);
}

bool pointInsideOrOnCircle(const geometry_msgs::Point& point, const geometry_msgs::Point& circle_center, const float& radius)
{
    float dx = abs(point.x - circle_center.x);
    float dy = abs(point.y - circle_center.y);

    if(dx + dy <= radius)
        return true;
    if(dx > radius)
        return false;
    if(dy > radius)
        return false;
    if(dx*dx + dy*dy <= radius*radius)
        return true;
    else
        return false;
}

bool isPointInPolygon(const rose_geometry::Point& point, const vector<rose_geometry::Point>& poly_points) 
{
    int i, j, nvert = poly_points.size();
    bool c = false;

    for(i = 0, j = nvert - 1; i < nvert; j = i++) 
    {
        if( ( (poly_points[i].y > point.y ) != (poly_points[j].y > point.y) ) &&
            (point.x < (poly_points[j].x - poly_points[i].x) * (point.y - poly_points[i].y) / (poly_points[j].y - poly_points[i].y) + poly_points[i].x)
          )
            c = !c;
    }

    return c;
}

//! @todo make unit tests
vector<rose_geometry::Point> intersectionsLineSegmentLineSegment(     const rose_geometry::Point& l1a, const rose_geometry::Point& l1b, 
                                                                                const rose_geometry::Point& l2a, const rose_geometry::Point& l2b)

{
    vector<rose_geometry::Point> intersection_points;

    // Suppose the two line segments run from p to p + r and from q to q + s. 
    // Then any point on the first line is representable as p + t r (for a scalar parameter t) 
    // and any point on the second line as q + u s (for a scalar parameter u).

    rose_geometry::Point p = l1a;
    rose_geometry::Point r = l1b - l1a;
    rose_geometry::Point q = l2a;
    rose_geometry::Point s = l2b - l2a;

    // Check if all points are the same
    if(l1a == l1b || l2a == l2b)
        return intersection_points;

    // If r x s = 0 and (q - p) x r = 0, then the two lines are collinear. If in addition, either 0 <= (q - p) * r <= r * r or 0 <= (p - q) * s <= s * s, then the two lines are overlapping.
    // If r x s = 0 and (q - p) x r = 0, but neither 0 <= (q - p) * r <= r * r nor 0 <= (p - q) * s <= s * s, then the two lines are collinear but disjoint.
    // If r x s = 0 and (q - p) x r != 0, then the two lines are parallel and non-intersecting.
    // If r x s != 0 and 0 <= t <= 1 and 0 <= u <= 1, the two line segments meet at the point p + t r = q + u s.

    // Otherwise, the two line segments are not parallel but do not intersect.
    else if(r.cross_2d(s) == 0.0)
    {
        // Lines are colinear, check if they overlap
        if((q - p).cross_2d(r) == 0.0)
        {
            // If the lines overlap there are infinite intersection points 
            // Therefore only report the overlapping endpoints
            if(isBetweenOrEqual(l1a.x, l2a.x, l2b.x) && isBetweenOrEqual(l1a.y, l2a.y, l2b.y))
                intersection_points.push_back(l1a.getROSmsg());
            
            if(isBetweenOrEqual(l1b.x, l2a.x, l2b.x) && isBetweenOrEqual(l1b.y, l2a.y, l2b.y))
                intersection_points.push_back(l1b.getROSmsg());
            
            if(isBetweenOrEqual(l2a.x, l1a.x, l1b.x) && isBetweenOrEqual(l2a.y, l1a.y, l1b.y))
                intersection_points.push_back(l2a.getROSmsg());
            
            if(isBetweenOrEqual(l2b.x, l1a.x, l1b.x) && isBetweenOrEqual(l2b.y, l1a.y, l1b.y))
                intersection_points.push_back(l2b.getROSmsg());     
        }
    }
    else 
    {
        // Lines are not collinear, check if intersecting
        float t = (q - p).cross_2d(s) / (r.cross_2d(s));
        float u = (q - p).cross_2d(r) / (r.cross_2d(s));

        if((0.0 <= t) && (t <= 1.0) && (0.0 <= u) && (u <= 1.0))
        {
            // Intersection at the point p + t r = q + u s.
            rose_geometry::Point intersection_point = p + (t*r);
            intersection_points.push_back(intersection_point.getROSmsg());
        }
        // else no intersection
    }

    return intersection_points;
}

vector<rose_geometry::Point> intersectionsLineSegmentCircle(  const rose_geometry::Point& linepoint_a, 
                                                                        const rose_geometry::Point& linepoint_b, 
                                                                        const rose_geometry::Point& circle_center, 
                                                                        const float& r)
{
    rose_geometry::Point intersection_point;
    vector<rose_geometry::Point> intersection_points;

    // If r equals zero check if the circle_center is on the line, if so return it, otherwise there will be no intersections.
    if(r == 0)
    {
        if(distancePointLine(circle_center, linepoint_a, linepoint_b) == 0)
            intersection_points.push_back(circle_center);

        return intersection_points;
    }

    // Radius squared
    float r_sq      = r*r;

    // Radi from center to start and end points
    float dist_s_sq     = distanceSq(circle_center, linepoint_a);
    float dist_e_sq     = distanceSq(circle_center, linepoint_b);
    float max_dist_sq   = std::fmax(dist_s_sq, dist_e_sq);
    
    // Check if circle is around both points, in this case there are no intersection points.
    if(r_sq > max_dist_sq)
        return intersection_points;

    // Get the perpendicular distance of the point and the line (shortest distance to).
    float perp_dist = distancePointLine(circle_center, linepoint_a, linepoint_b);

    // Check if circle is so small it does not touch the line
    if(r < fabs(perp_dist))
        return intersection_points;

    // Calculate the angle alpha which is the angle from the vector A to the vector B
    // Calculate the angle beta which is the angle from the x axis to the vector A
    // Where A is the vector from the circle center to the perpendicular projection of the center on the lin (pd)
    // Where B is the vector from the circle center to a possible intersection point (pi)
    // float li            = sqrt(r*r - perp_dist*perp_dist);
    float alpha           = acos(-perp_dist/r);
    float line_angle_a    = angle(linepoint_a, linepoint_b);
    float line_angle_b    = angle(linepoint_b, linepoint_a);
    float line_angle      = angle(linepoint_a, linepoint_b);
    // if(fabs(line_angle_a) <= fabs(line_angle_b))
    //     line_angle = line_angle_a;
    // else
    //     line_angle = line_angle_b;

    float beta          = wrapToPi(line_angle - 0.5*M_PI);

    
    // Two possible intersection points, however they may lie on the infinite line discribed by the line segment.
    // Therefore we need an extra test to test if they are indeed on the line segment.
    float y1 = wrapToPi(beta - alpha); // iff r_sq < dist_s_sq
    float y2 = wrapToPi(beta + alpha); // iff r_sq < dist_e_sq

    // Create and return the valid intersection points
    if(r_sq <= dist_s_sq)
    {
        intersection_point.x = r;
        intersection_point.y = 0.0;
        rotateVect(&intersection_point.x, &intersection_point.y, y1);
        intersection_point.x += circle_center.x;
        intersection_point.y += circle_center.y;
        intersection_points.push_back(intersection_point);
    }

    // Also check if perp_dist is equal to the radius, becaus then there is only one solution
    if(r_sq <= dist_e_sq && r != fabs(perp_dist))
    {
        intersection_point.x = r;
        intersection_point.y = 0.0;
        rotateVect(&intersection_point.x, &intersection_point.y, y2);
        intersection_point.x += circle_center.x;
        intersection_point.y += circle_center.y;
        intersection_points.push_back(intersection_point);
    }

    return intersection_points;
}

// Calculate perpendicular distance of point p to line discribed by linepoint_a and linepoint_b
// Infinite line (not segment), returns 0 if linepoints a and b are the same
// Returns signed distance, positive distance on the left side and negative on the right
float distancePointLine(const geometry_msgs::Point& p, const geometry_msgs::Point& linepoint_a, const geometry_msgs::Point& linepoint_b)
{
    float dx = linepoint_b.x - linepoint_a.x;
    float dy = linepoint_b.y - linepoint_a.y;

    // If the line is just a point, it has no direction, return  0.0
    if(dx == 0.0 && dy == 0.0)
        return 0.0;

    float numerator   = dy*p.x - dx*p.y - linepoint_a.x*linepoint_b.y + linepoint_b.x*linepoint_a.y;
    return numerator/sqrt(dx*dx + dy*dy);
}

float distancePointLine(    const rose_geometry::Point& p, 
                            const rose_geometry::Point& linepoint_a, 
                            const rose_geometry::Point& linepoint_b)
{
    return distancePointLine(p.getROSmsg(), linepoint_a.getROSmsg(), linepoint_b.getROSmsg());
}

float minimalDistancePointPolygon( const rose_geometry::Point& p, const vector<rose_geometry::Point>& poly_points)
{   
    unsigned int i;
    float min_dist = std::numeric_limits<float>::max();
    for(i = 0; i < poly_points.size() - 2; i++)
    {
        float dist = distancePointLine(p, poly_points[i], poly_points[i+1]);
        if(dist < min_dist)
            min_dist = dist;
    }
    return min_dist;
}                                    

//! @todo: Make unit tests
float dot(const Point& a, const Point& b)
{
    return a.x*b.x + a.y*b.y;
}
//! @todo: Make unit tests
float cross(const Point& a, const Point& b)
{
    return a.x*b.y - a.y*b.x;
}

// Check if point a, b and c are colinear
//! @todo: Make unit tests
bool aligned(const rose_geometry::Point& a, const rose_geometry::Point&b, const rose_geometry::Point& c)
{
    // Check if the cross product of (b-a) and (c-a) is 0. 
    // Use floating point precicion epsilon
    return (b - a).cross_2d(c - a) < std::numeric_limits<float>::epsilon(); 
}

// Check if point c is colinear with and inbetween points a and b
//! @todo: Make unit tests
bool isBetween(const rose_geometry::Point& a, const rose_geometry::Point&b, const rose_geometry::Point& c)
{
    // Check if aligned and if the dot product of (b-a) and (c-a) is positive and is less than the square of the distance between a and b.
    float dotProduct = (b - a).dot(c - a);
    return aligned(a, b, c) && dotProduct > 0.0 && dotProduct < distance(a, b);
}

// Check if a is between b and c
bool isBetweenOrEqual(const float& a, const float&b, const float& c)
{
    if((a >= b && a <= c) || (a >= c && a <= b))
        return true;
    else
        return false;
}

geometry_msgs::PoseStamped translatePose(const geometry_msgs::PoseStamped& stamped_pose, const geometry_msgs::Vector3& translation)
{
    geometry_msgs::PoseStamped translated_stamped_pose = stamped_pose;
    translated_stamped_pose.pose.position.x += translation.x;
    translated_stamped_pose.pose.position.y += translation.y;
    translated_stamped_pose.pose.position.z += translation.z;
    
    return translated_stamped_pose;
}

geometry_msgs::PoseStamped rotatePose(const geometry_msgs::PoseStamped& stamped_pose, const tf::Quaternion& quat_tf_rotation)
{
    tf::Quaternion quat_tf_pose, quat_tf_rotated;
    quaternionMsgToTF(stamped_pose.pose.orientation, quat_tf_pose);

    // Do actually rotate
    quat_tf_rotated = quat_tf_pose*quat_tf_rotation;

    geometry_msgs::PoseStamped rotated_stamped_pose = stamped_pose;
    quaternionTFToMsg(quat_tf_rotated, rotated_stamped_pose.pose.orientation);

    return rotated_stamped_pose;
}

geometry_msgs::PoseStamped rotatePose(const geometry_msgs::PoseStamped& stamped_pose, const geometry_msgs::Quaternion& quat_rotation_msg)
{
    tf::Quaternion quat_tf_rotation;
    quaternionMsgToTF(quat_rotation_msg, quat_tf_rotation);

    return rotatePose(stamped_pose, quat_tf_rotation);
}

void rotatePointAroundOrigin(geometry_msgs::Point& point, float radians)
{
    float cs = cos(radians);
    float sn = sin(radians);
    float px = point.x * cs - point.y * sn; 
    float py = point.x * sn + point.y * cs;
    
    point.x = px;
    point.y = py;
}

void rotatePointAroundOrigin(rose_geometry::Point& point, float radians)
{
    float cs = cos(radians);
    float sn = sin(radians);
    float px = point.x * cs - point.y * sn; 
    float py = point.x * sn + point.y * cs;
    
    point.x = px;
    point.y = py;
}

//! @todo OH: Put radians as last argument
void rotatePointsAroundOrigin(float radians, std::vector<geometry_msgs::Point>& points)
{
    float cs = cos(radians);
    float sn = sin(radians);
    float px = 0.0;
    float py = 0.0;

    for(auto& point : points)
    {
        px = point.x * cs - point.y * sn; 
        py = point.x * sn + point.y * cs;
        
        point.x = px;
        point.y = py;
    }
}

//! @todo OH: Put radians as last argument
void rotatePointsAroundOrigin(float radians, std::vector<rose_geometry::Point>& points)
{
    float cs = cos(radians);
    float sn = sin(radians);
    float px = 0.0;
    float py = 0.0;

    for(auto& point : points)
    {
        px = point.x * cs - point.y * sn; 
        py = point.x * sn + point.y * cs;
        
        point.x = px;
        point.y = py;
    }
}

geometry_msgs::Vector3 rotate2DVector(const geometry_msgs::Vector3& vect, float radians)
{
    geometry_msgs::Vector3 rotated_vector;
    float cs = cos(radians);
    float sn = sin(radians);
    float px = vect.x * cs - vect.y * sn; 
    float py = vect.x * sn + vect.y * cs;
    
    rotated_vector.x = px;
    rotated_vector.y = py;

    return rotated_vector;
}

void translatePoint(const float& dx, const float& dy, geometry_msgs::Point& point)
{
    point.x += dx;
    point.y += dy;
}

void translatePoint(const float& dx, const float& dy, rose_geometry::Point& point)
{
    point.x += dx;
    point.y += dy;
}

void translatePoints(const float& dx, const float& dy, std::vector<geometry_msgs::Point>& points)
{
    for(auto& point : points)
    {
        point.x += dx;
        point.y += dy;
    }
}

void translatePoints(const float& dx, const float& dy, std::vector<rose_geometry::Point>& points)
{
    for(auto& point : points)
    {
        point.x += dx;
        point.y += dy;
    }
}

// Check if the shortest rotation between two vectors is CW or CCW
bool areClockwise(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2) 
{
    return -v1.x*v2.y + v1.y*v2.x > 0.0;
}

} // namespace rose_geometry
