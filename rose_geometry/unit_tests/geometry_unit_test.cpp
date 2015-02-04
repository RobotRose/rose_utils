/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2014/07/17
* 		- File created.
*
* Description:
*	Unit tests for the common functions, http://www.ibm.com/developerworks/aix/library/au-googletestingframework.html
* 
***********************************************************************************/

// Bring in my package's API, which is what I'm testing
#include "rose_common/common.hpp"
// Bring in gtest
#include <gtest/gtest.h>


// Extend the geometry_msgs::Point class to be able to print with gtest
namespace geometry_msgs {

// It's important that PrintTo() is defined in the SAME
// namespace that defines Point.  C++'s look-up rules rely on that.
void PrintTo(const Point& p, ::std::ostream* os) 
{
	*os << "[" << p.x << ", " << p.y << "]";
}
}

TEST(geometry, Point)
{
	rose_geometry::Point a;
	EXPECT_FLOAT_EQ(0.0 , a.x);
	EXPECT_FLOAT_EQ(0.0 , a.y);
	EXPECT_FLOAT_EQ(0.0 , a.z);

	rose_geometry::Point b(0.1, -12.345, 1e6);
	EXPECT_FLOAT_EQ(0.1 		, b.x);
	EXPECT_FLOAT_EQ(-12.345		, b.y);
	EXPECT_FLOAT_EQ(1e6 		, b.z);

	geometry_msgs::Point ros_point;
	EXPECT_FLOAT_EQ(0.0 , ros_point.x);
	EXPECT_FLOAT_EQ(0.0	, ros_point.y);
	EXPECT_FLOAT_EQ(0.0 , ros_point.z);

	ros_point.x = 1.0;
	ros_point.y = 2.0;
	ros_point.z = 3.0;
	EXPECT_FLOAT_EQ(1.0 , ros_point.x);
	EXPECT_FLOAT_EQ(2.0	, ros_point.y);
	EXPECT_FLOAT_EQ(3.0 , ros_point.z);

	a = ros_point;
	EXPECT_FLOAT_EQ(1.0 , a.x);
	EXPECT_FLOAT_EQ(2.0 , a.y);
	EXPECT_FLOAT_EQ(3.0 , a.z);

	rose_geometry::Point d(-1.0, -2.0, -3.0);
	EXPECT_FLOAT_EQ(-1.0 , d.x);
	EXPECT_FLOAT_EQ(-2.0 , d.y);
	EXPECT_FLOAT_EQ(-3.0 , d.z);

	rose_geometry::Point e;
	e -= d;
	EXPECT_FLOAT_EQ(-1.0 , d.x);
	EXPECT_FLOAT_EQ(-2.0 , d.y);
	EXPECT_FLOAT_EQ(-3.0 , d.z);

	e = rose_geometry::Point(0.0, 0.0, 0.0);
	e = e - d;
	EXPECT_FLOAT_EQ(1.0 , e.x);
	EXPECT_FLOAT_EQ(2.0 , e.y);
	EXPECT_FLOAT_EQ(3.0 , e.z);

	e = rose_geometry::Point(-1.0, -2.0, -3.0);
	e = e - d;
	EXPECT_FLOAT_EQ(0.0 , e.x);
	EXPECT_FLOAT_EQ(0.0 , e.y);
	EXPECT_FLOAT_EQ(0.0 , e.z);

	e = rose_geometry::Point(0.0, 0.0, 0.0);
	d = rose_geometry::Point(5.5, -2.0, 3.0);
	e += d;
	EXPECT_FLOAT_EQ(5.5 , e.x);
	EXPECT_FLOAT_EQ(-2.0 , e.y);
	EXPECT_FLOAT_EQ(3.0 , e.z);

	e = d + e;
	EXPECT_FLOAT_EQ(11.0 , e.x);
	EXPECT_FLOAT_EQ(-4.0 , e.y);
	EXPECT_FLOAT_EQ(6.0 , e.z);

	e = rose_geometry::Point(2.0, 2.0, 2.0);
	d = rose_geometry::Point(2.0, -2.0, 0.0);
	e *= d;
	EXPECT_FLOAT_EQ(4.0 , e.x);
	EXPECT_FLOAT_EQ(-4.0 , e.y);
	EXPECT_FLOAT_EQ(0.0 , e.z);

	e = d * e;
	EXPECT_FLOAT_EQ(8.0 , e.x);
	EXPECT_FLOAT_EQ(8.0 , e.y);
	EXPECT_FLOAT_EQ(0.0 , e.z);

	e = rose_geometry::Point(2.0, 4.0, 2.0);
	d = rose_geometry::Point(2.0, -2.0, 1.0);
	e /= d;
	EXPECT_FLOAT_EQ(1.0 , e.x);
	EXPECT_FLOAT_EQ(-2.0 , e.y);
	EXPECT_FLOAT_EQ(2.0 , e.z);

	e = d / e;
	EXPECT_FLOAT_EQ(2.0 , e.x);
	EXPECT_FLOAT_EQ(1.0 , e.y);
	EXPECT_FLOAT_EQ(0.5 , e.z);

	a = rose_geometry::Point(1.0, 1.0, 1.0);
	a += 2.35;
	EXPECT_FLOAT_EQ(3.35 , a.x);
	EXPECT_FLOAT_EQ(3.35 , a.y);
	EXPECT_FLOAT_EQ(3.35 , a.z);

	a = a + -2.35;
	EXPECT_FLOAT_EQ(1.0 , a.x);
	EXPECT_FLOAT_EQ(1.0 , a.y);
	EXPECT_FLOAT_EQ(1.0 , a.z);

	a = rose_geometry::Point(1.0, 2.0, 0.0);
	a *= 4.5;
	EXPECT_FLOAT_EQ(4.5 , a.x);
	EXPECT_FLOAT_EQ(9.0 , a.y);
	EXPECT_FLOAT_EQ(0.0 , a.z);

	a = a * (1.0/4.5);
	EXPECT_FLOAT_EQ(1.0 , a.x);
	EXPECT_FLOAT_EQ(2.0 , a.y);
	EXPECT_FLOAT_EQ(0.0 , a.z);

	a = rose_geometry::Point(1.0, 2.0, 0.0);
	a -= 2.0;
	EXPECT_FLOAT_EQ(-1.0 , a.x);
	EXPECT_FLOAT_EQ(0.0 , a.y);
	EXPECT_FLOAT_EQ(-2.0 , a.z);

	a = a - 5.0;
	EXPECT_FLOAT_EQ(-6.0 , a.x);
	EXPECT_FLOAT_EQ(-5.0 , a.y);
	EXPECT_FLOAT_EQ(-7.0 , a.z);

	a = rose_geometry::Point(1.0, 2.0, 0.0);
	a /= 2.0;
	EXPECT_FLOAT_EQ(0.5 , a.x);
	EXPECT_FLOAT_EQ(1.0 , a.y);
	EXPECT_FLOAT_EQ(0.0 , a.z);

	a = a / 0.5;
	EXPECT_FLOAT_EQ(1.0 , a.x);
	EXPECT_FLOAT_EQ(2.0 , a.y);
	EXPECT_FLOAT_EQ(0.0 , a.z);

	a = rose_geometry::Point(1.0, 1.0, 1.0);
	b = rose_geometry::Point(1.0, 1.0, 1.0);
	rose_geometry::Point c = rose_geometry::Point(1.0, 1.0, 1.0);
	d = rose_geometry::Point(1.0, 1.0, 1.0);
	e = (((a+b)*2.0)-4.0)+2.0 / ((a+b)*(c+d))/2.0;
	EXPECT_FLOAT_EQ(1.0 , e.x);
	EXPECT_FLOAT_EQ(1.0 , e.y);
	EXPECT_FLOAT_EQ(1.0 , e.z);

}

//! @todo OH: Finish this unit test
TEST(rose_geometry, intersectionsLineSegmentLineSegment)
{
	rose_geometry::Point s1, e1, s2, e2;

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(2.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 2.0, 0.0);
	e2 = rose_geometry::Point(2.0, 2.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(1.0, 2.0, 0.0);
	s2 = rose_geometry::Point(2.0, 1.0, 0.0);
	e2 = rose_geometry::Point(2.0, 2.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(2.0, 1.0, 0.0);
	e1 = rose_geometry::Point(3.0, 2.0, 0.0);
	s2 = rose_geometry::Point(-2.0, -1.0, 0.0);
	e2 = rose_geometry::Point(-3.0, -2.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(0.0, 1.0, 0.0);
	e1 = rose_geometry::Point(2.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 3.0, 0.0);
	e2 = rose_geometry::Point(1.0, 4.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(1.0, 1.0, 0.0);
	s2 = rose_geometry::Point(-2.0, 0.0, 0.0);
	e2 = rose_geometry::Point(-2.0, 4.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(-1.0, -1.0, 0.0);
	e1 = rose_geometry::Point(-1.0, -1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 1.0, 0.0);
	e2 = rose_geometry::Point(1.0, 1.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(2.0, 1.0, 0.0);
	s2 = rose_geometry::Point(4.0, 1.0, 0.0);
	e2 = rose_geometry::Point(5.0, 1.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
		
	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(2.0, 1.0, 0.0);
	s2 = rose_geometry::Point(5.0, 1.0, 0.0);
	e2 = rose_geometry::Point(4.0, 1.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
		
	s1 = rose_geometry::Point(5.0, 1.0, 0.0);
	e1 = rose_geometry::Point(4.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 1.0, 0.0);
	e2 = rose_geometry::Point(2.0, 1.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(-1.0, 2.0, 0.0);
	s2 = rose_geometry::Point(-1.0, 4.0, 0.0);
	e2 = rose_geometry::Point(-1.0, 5.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
		
	s1 = rose_geometry::Point(-1.0, 2.0, 0.0);
	e1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	s2 = rose_geometry::Point(-1.0, 5.0, 0.0);
	e2 = rose_geometry::Point(-1.0, 4.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());

	s1 = rose_geometry::Point(1.0, 2.0, 0.0);
	e1 = rose_geometry::Point(2.0, 2.0, 0.0);
	s2 = rose_geometry::Point(2.0, 1.0, 0.0);
	e2 = rose_geometry::Point(2.0, 2.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 2.0, 0.0);
	e1 = rose_geometry::Point(1.0, -2.0, 0.0);
	s2 = rose_geometry::Point(1.0, -2.0, 0.0);
	e2 = rose_geometry::Point(2.0, -2.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(-2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 2.0, 0.0);
	e1 = rose_geometry::Point(3.0, 2.0, 0.0);
	s2 = rose_geometry::Point(2.0, 1.0, 0.0);
	e2 = rose_geometry::Point(2.0, 3.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(-1.0, -1.0, 0.0);
	s2 = rose_geometry::Point(-2.0, 2.0, 0.0);
	e2 = rose_geometry::Point(2.0, -2.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(1.0, 3.0, 0.0);
	s2 = rose_geometry::Point(1.0, 2.0, 0.0);
	e2 = rose_geometry::Point(2.0, 2.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(-3.0, 1.0, 0.0);
	s2 = rose_geometry::Point(-2.0, 1.0, 0.0);
	e2 = rose_geometry::Point(-2.0, 2.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(-2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(0.0, 1.0, 0.0);
	s2 = rose_geometry::Point(0.0, 1.0, 0.0);
	e2 = rose_geometry::Point(1.0, 2.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(0.0, 1.0, 0.0);
	e1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 2.0, 0.0);
	e2 = rose_geometry::Point(0.0, 1.0, 0.0);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);

	s1 = rose_geometry::Point(-1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(2.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 1.0, 0.0);
	e2 = rose_geometry::Point(3.0, 1.0, 0.0);
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[1].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[1].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(4.0, 1.0, 0.0);
	s2 = rose_geometry::Point(2.0, 1.0, 0.0);
	e2 = rose_geometry::Point(3.0, 1.0, 0.0);
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);
	EXPECT_NEAR(3.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[1].x, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[1].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 3.0, 0.0);
	e1 = rose_geometry::Point(1.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 3.0, 0.0);
	e2 = rose_geometry::Point(1.0, 2.0, 0.0);
	ASSERT_EQ(3, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].x, 1e-5);
	EXPECT_NEAR(3.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[0].y, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[1].x, 1e-5);
	EXPECT_NEAR(3.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[1].y, 1e-5);
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[2].x, 1e-5);
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2)[2].y, 1e-5);

	s1 = rose_geometry::Point(1.0, 1.0, 0.0);
	e1 = rose_geometry::Point(1.0, 1.0, 0.0);
	s2 = rose_geometry::Point(1.0, 1.0, 0.0);
	e2 = rose_geometry::Point(1.0, 1.0, 0.0);
	ASSERT_EQ(0, rose_geometry::intersectionsLineSegmentLineSegment(s1, e1, s2, e2).size());



}

TEST(rose_geometry, distanceXY)
{
	Point a, b;

	#define PrintIt 	<< "a = " << ::testing::PrintToString(a) << "\nb = " << ::testing::PrintToString(b)
	
	a.x = 0.0;
	a.y = 0.0;
	b.x = 0.0;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b))  PrintIt;

	a.x = 1.0;
	a.y = 0.0;
	b.x = 0.0;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 0.0;
	a.y = 1.0;
	b.x = 0.0;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 0.0;
	a.y = 0.0;
	b.x = 1.0;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 0.0;
	a.y = 0.0;
	b.x = 0.0;
	b.y = 1.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = -1.0;
	a.y = 0.0;
	b.x = -1.0;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 1.0;
	a.y = 1.0;
	b.x = 0.0;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 0.0;
	a.y = 0.0;
	b.x = 1.0;
	b.y = 1.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 1.0;
	a.y = 1.0;
	b.x = 1.0;
	b.y = 1.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = -1.0;
	a.y = -1.0;
	b.x = -1.0;
	b.y = -1.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = 10.0;
	a.y = 12.0;
	b.x = 13.0;
	b.y = 14.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	a.x = -3.5;
	a.y = -6.5;
	b.x = 4.7;
	b.y = 0.0;
	EXPECT_FLOAT_EQ(sqrt(pow(b.x - a.x, 2.0) + pow(b.y - a.y, 2.0)) , rose_geometry::distanceXY(a, b)) PrintIt;

	#undef PrintIt
}

TEST(rose_geometry, distancePointLine)
{
	Point start, end, point;

	#define PrintIt 	<< "start = " << ::testing::PrintToString(start) << "\nend = " << ::testing::PrintToString(end)  << "\npoint = " << ::testing::PrintToString(point)			

	// Line is just a point -1 expected
	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 0.0;
	end.y		= 0.0;
	point.x		= 0.0;
	point.y		= 0.0;
	EXPECT_FLOAT_EQ(0.0, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= -1.0;
	start.y 	= 0.0;
	end.x 		= 1.0;
	end.y		= 0.0;
	point.x		= 0.0;
	point.y		= 0.0;
	EXPECT_FLOAT_EQ(0.0, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= 0.0;
	start.y 	= -1.0;
	end.x 		= 0.0;
	end.y		= 1.0;
	point.x		= 0.0;
	point.y		= 0.0;
	EXPECT_FLOAT_EQ(0.0, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= 1.0;
	start.y 	= -1.0;
	end.x 		= -1.0;
	end.y		= 1.0;
	point.x		= 0.0;
	point.y		= 0.0;
	EXPECT_FLOAT_EQ(0.0, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= 0.0;
	start.y 	= 10.0;
	end.x 		= 0.0;
	end.y		= -10.0;
	point.x		= 5.5;
	point.y		= 10.0;
	EXPECT_FLOAT_EQ(-5.5, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= 0.0;
	start.y 	= -10.0;
	end.x 		= 0.0;
	end.y		= 10.0;
	point.x		= 5.5;
	point.y		= 10.0;
	EXPECT_FLOAT_EQ(5.5, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= 0.0;
	start.y 	= 10.0;
	end.x 		= 0.0;
	end.y		= -10.0;
	point.x		= 5.5;
	point.y		= 2.0;
	EXPECT_FLOAT_EQ(-5.5, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	start.x 	= 0.0;
	start.y 	= 10.0;
	end.x 		= 0.0;
	end.y		= -10.0;
	point.x		= 5.5;
	point.y		= 0.0;
	EXPECT_FLOAT_EQ(-5.5, rose_geometry::distancePointLine(point, start, end)) PrintIt;

	#undef PrintIt
}

TEST(rose_geometry, intersectionsLineSegmentCircleNoIntersections)
{
	rose_geometry::Point start, end, center;
	float radius;

	#define PrintIt 	<< "start = " << ::testing::PrintToString(start) << "\nend = " << ::testing::PrintToString(end)  << "\ncenter = " << ::testing::PrintToString(center) << "\nradius = " << radius				



	start.x 	= 1.0;
	start.y 	= 1.0;
	end.x 		= 1.0;
	end.y		= -1.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= 0.5;
	EXPECT_EQ(0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;

	start.x 	= 1.0;
	start.y 	= 1.0;
	end.x 		= 1.0;
	end.y		= -1.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= 10.0;
	EXPECT_EQ(0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;

	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 1.0;
	end.y		= 0.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= 1.5;
	EXPECT_EQ(0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;

	start.x 	= 3.0;
	start.y 	= 4.0;
	end.x 		= 3.0;
	end.y		= 2.0;
	center.x	= 3.0;
	center.y	= 3.0;
	radius		= 2.0;
	EXPECT_EQ(0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;

	start.x 	= 4.0;
	start.y 	= 3.0;
	end.x 		= 2.0;
	end.y		= 3.0;
	center.x	= 3.0;
	center.y	= 3.0;
	radius		= 2.0;
	EXPECT_EQ(0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;

	#undef PrintIt
}

TEST(rose_geometry, intersectionsLineSegmentCircleOneIntersection)
{
	rose_geometry::Point start, end, center;
	float radius;

	#define PrintIt 	<< "start = " << ::testing::PrintToString(start) << "\nend = " << ::testing::PrintToString(end)  << "\ncenter = " << ::testing::PrintToString(center) << "\nradius = " << radius				
	
	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 0.0;
	end.y		= 0.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= 0.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 1.0;
	start.y 	= 1.0;
	end.x 		= -1.0;
	end.y		= -1.0;
	center.x	= 0.5;
	center.y	= 0.5;
	radius		= 0.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= -1.0;
	start.y 	= -1.0;
	end.x 		= 1.0;
	end.y		= 1.0;
	center.x	= -1.0;
	center.y	= -1.0;
	radius		= 0.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(-1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(-1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 1.0;
	start.y 	= 1.0;
	end.x 		= -1.5;
	end.y		= -1.5;
	center.x	= -1.5;
	center.y	= -1.5;
	radius		= 0.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(-1.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(-1.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 1.0;
	end.y		= 0.0;
	center.x	= 1.0;
	center.y	= 0.0;
	radius		= 0.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 0.0;
	end.y		= 1.0;
	center.x	= 0.0;
	center.y	= 1.0;
	radius		= 0.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 1.0;
	end.y		= 0.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= 0.5;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= -10.0;
	start.y 	= 0.0;
	end.x 		= 10.0;
	end.y		= 0.0;
	center.x	= 0.0;
	center.y	= 10.0;
	radius		= 10.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;


	start.x 	= -9.0;
	start.y 	= 0.0;
	end.x 		= 9.0;
	end.y		= 0.0;
	center.x	= -9.0;
	center.y	= 9.0;
	radius		= 9.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(-9.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= -8.0;
	start.y 	= 0.0;
	end.x 		= 8.0;
	end.y		= 0.0;
	center.x	= 8.0;
	center.y	= 8.0;
	radius		= 8.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(8.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 0.0;
	start.y 	= -7.0;
	end.x 		= 0.0;
	end.y		= 7.0;
	center.x	= 7.0;
	center.y	= -7.0;
	radius		= 7.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(-7.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 5.0;
	start.y 	= 0.0;
	end.x 		= 5.0;
	end.y		= 10.0;
	center.x	= 3.0;
	center.y	= 3.0;
	radius		= 2.0;
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(5.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(3.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	start.x 	= 2.0;
	start.y 	= 2.0;
	end.x 		= 1.0;
	end.y		= 1.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= sqrt(1.5*1.5 + 1.5*1.5);
	ASSERT_EQ(1, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(1.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(1.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;

	#undef PrintIt
}


TEST(rose_geometry, intersectionsLineSegmentCircleTwoIntersections)
{
	rose_geometry::Point start, end, center;
	float radius;

	#define PrintIt 	<< "start = " << ::testing::PrintToString(start) << "\nend = " << ::testing::PrintToString(end)  << "\ncenter = " << ::testing::PrintToString(center) << "\nradius = " << radius				

	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 1.0;
	end.y		= 0.0;
	center.x	= 0.25;
	center.y	= 0.00;
	radius		= 0.25;
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;
	EXPECT_NEAR(0.5, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].y, 1e-5) PrintIt;

	start.x 	= 0.0;
	start.y 	= 0.0;
	end.x 		= 1.0;
	end.y		= 0.0;
	center.x	= 0.5;
	center.y	= 0.0;
	radius		= 0.25;
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(0.25, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;
	EXPECT_NEAR(0.75, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].x, 1e-5) PrintIt;
	EXPECT_NEAR(0.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].y, 1e-5) PrintIt;

	start.x 	= -1.0;
	start.y 	= -1.0;
	end.x 		= 1.0;
	end.y		= 1.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= sqrt(1.0*1.0 + 1.0*1.0);
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(-1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(-1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].x, 1e-5) PrintIt;
	EXPECT_NEAR(1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].y, 1e-5) PrintIt;

// here
	start.x 	= -9.0;
	start.y 	= -9.0;
	end.x 		= 11.0;
	end.y		= 11.0;
	center.x	= 1.0;
	center.y	= 1.0;
	radius		= sqrt(10.0*10.0 + 10.0*10.0);
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	EXPECT_NEAR(-9.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	EXPECT_NEAR(-9.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;
	EXPECT_NEAR(11.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].x, 1e-5) PrintIt;
	EXPECT_NEAR(11.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].y, 1e-5) PrintIt;

	start.x 	= -2.0;
	start.y 	= -2.0;
	end.x 		= 2.0;
	end.y		= -1.0;
	center.x	= 0.0;
	center.y	= 0.0;
	radius		= sqrt(5);
	ASSERT_EQ(2, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius).size()) PrintIt;
	// Dont know first point?
	// EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].x, 1e-5) PrintIt;
	// EXPECT_NEAR(-1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[0].y, 1e-5) PrintIt;
	EXPECT_NEAR(2.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].x, 1e-5) PrintIt;
	EXPECT_NEAR(-1.0, rose_geometry::intersectionsLineSegmentCircle(start, end, center, radius)[1].y, 1e-5) PrintIt;

	

	#undef PrintIt
}

TEST(rose_geometry, wrapToPi)
{
	EXPECT_FLOAT_EQ(0.0, rose_geometry::wrapToPi(0.0));
	EXPECT_FLOAT_EQ(0.5, rose_geometry::wrapToPi(0.5));
	EXPECT_FLOAT_EQ(1.0, rose_geometry::wrapToPi(1.0));
	EXPECT_FLOAT_EQ(-0.5, rose_geometry::wrapToPi(-0.5));
	EXPECT_FLOAT_EQ(-1.0, rose_geometry::wrapToPi(-1.0));
	EXPECT_FLOAT_EQ(-M_PI, rose_geometry::wrapToPi(M_PI));
	EXPECT_FLOAT_EQ(M_PI, rose_geometry::wrapToPi(-M_PI));

	EXPECT_NEAR(-M_PI + 0.1, rose_geometry::wrapToPi(M_PI + 0.1), 1e-5);
	EXPECT_NEAR( M_PI - 0.1, rose_geometry::wrapToPi(M_PI - 0.1), 1e-5);

	EXPECT_NEAR(0.1, rose_geometry::wrapToPi(2.0*M_PI + 0.1), 1e-5);
	EXPECT_NEAR(-0.1, rose_geometry::wrapToPi(2.0*M_PI - 0.1), 1e-5);

	EXPECT_NEAR(-M_PI + 0.1, rose_geometry::wrapToPi(3.0*M_PI + 0.1), 1e-5);
	EXPECT_NEAR( M_PI - 0.1, rose_geometry::wrapToPi(3.0*M_PI - 0.1), 1e-5);

	EXPECT_NEAR(0.1, rose_geometry::wrapToPi(4.0*M_PI + 0.1), 1e-5);
	EXPECT_NEAR(-0.1, rose_geometry::wrapToPi(4.0*M_PI - 0.1), 1e-5);


	EXPECT_NEAR(-M_PI + 0.1, rose_geometry::wrapToPi(-M_PI + 0.1), 1e-5);
	EXPECT_NEAR( M_PI - 0.1, rose_geometry::wrapToPi(-M_PI - 0.1), 1e-5);

	EXPECT_NEAR( 0.1, rose_geometry::wrapToPi(-2.0*M_PI + 0.1), 1e-5);
	EXPECT_NEAR(-0.1, rose_geometry::wrapToPi(-2.0*M_PI - 0.1), 1e-5);

	EXPECT_NEAR(-M_PI + 0.1, rose_geometry::wrapToPi(-3.0*M_PI + 0.1), 1e-5);
	EXPECT_NEAR( M_PI - 0.1, rose_geometry::wrapToPi(-3.0*M_PI - 0.1), 1e-5);	

	EXPECT_NEAR( 0.1, rose_geometry::wrapToPi(-4.0*M_PI + 0.1), 1e-5);
	EXPECT_NEAR(-0.1, rose_geometry::wrapToPi(-4.0*M_PI - 0.1), 1e-5);	
}

// // Declare a test
// TEST(TestSuite, testCase1)
// {
// 	<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// // Declare another test
// TEST(TestSuite, testCase2)
// {
// 	<test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
