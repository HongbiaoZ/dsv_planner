/**************************************************************************
misc_utils.h
Miscellaneous utility functions

Chao Cao (ccao1@andrew.cmu.edu)
6/5/19
**************************************************************************/
#ifndef MISC_UTILS_MISC_UTILS_H
#define MISC_UTILS_MISC_UTILS_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <math.h>

namespace misc_utils_ns
{
typedef pcl::PointXYZI PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLPointCloudType;
const double PI = 3.14159265358;

// Utility functions
template <class FromPointType, class ToPointType>
void PointToPoint(const FromPointType& from_point, ToPointType& to_point)
{
  to_point.x = from_point.x;
  to_point.y = from_point.y;
  to_point.z = from_point.z;
}
geometry_msgs::Point PCL2GeoMsgPnt(const PCLPointType& pnt);
PCLPointType GeoMsgPnt2PCL(const geometry_msgs::Point& pnt);
geometry_msgs::Point GeoMsgPoint(double x, double y, double z);
PCLPointType PCLPoint(float x, float y, float z);
void LeftRotatePoint(PCLPointType& pnt);
void RightRotatePoint(PCLPointType& pnt);
void LeftRotatePoint(geometry_msgs::Point& pnt);
void RightRotatePoint(geometry_msgs::Point& pnt);
template <class CloudType>
void KeyposeToMap(CloudType& cloud, const nav_msgs::Odometry::ConstPtr& keypose);
double PointXYDist(const geometry_msgs::Point& pnt1, const geometry_msgs::Point& pnt2);
double PointXYDist(const PCLPointType& pnt1, const PCLPointType& pnt2);
template <class P1, class P2>
double PointXYDist(const P1& pnt1, const P2& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2));
}
template <class P1, class P2>
double PointXYZDist(const P1& pnt1, const P2& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2) + pow((pnt1.z - pnt2.z), 2));
}
double PointAngle(const geometry_msgs::Point& pnt, const geometry_msgs::Point& robot_pos);
double PointAngle(const PCLPointType& pnt, const geometry_msgs::Point& robot_pos);
bool LineSegIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& q1, const geometry_msgs::Point& p2,
                      const geometry_msgs::Point& q2);
bool LineSegIntersectWithTolerance(const geometry_msgs::Point& p1, const geometry_msgs::Point& q1,
                                   const geometry_msgs::Point& p2, const geometry_msgs::Point& q2,
                                   const double tolerance);
int ThreePointOrientation(const geometry_msgs::Point& p, const geometry_msgs::Point& q, const geometry_msgs::Point& r);
bool PointOnLineSeg(const geometry_msgs::Point& p, const geometry_msgs::Point& q, const geometry_msgs::Point& r);
double AngleOverlap(double s1, double e1, double s2, double e2);
double AngleDiff(double source_angle, double target_angle);
bool PointInPolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
double LineSegDistance2D(const geometry_msgs::Point& point, const geometry_msgs::Point& line_segment_start,
                         const geometry_msgs::Point& line_segment_end);
double DistancePoint2DToPolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
template <class PCLPointType>
void LinInterpPoints(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double resolution,
                     typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
{
  double point_dist = PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(p1, p2);
  if (point_dist < 0.01)
    return;
  PCLPointType point1;
  point1.x = p1.x;
  point1.y = p1.y;
  point1.z = p1.z;
  PCLPointType point2;
  point2.x = p1.x;
  point2.y = p1.y;
  point2.z = p1.z;
  if (point_dist < resolution)
  {
    cloud->points.push_back(point1);
    cloud->points.push_back(point2);
  }
  else
  {
    int num_points = static_cast<int>(point_dist / resolution);
    cloud->points.push_back(point1);
    for (int i = 0; i < num_points; i++)
    {
      PCLPointType point;
      point.x = static_cast<float>((p2.x - p1.x) / num_points * i + p1.x);
      point.y = static_cast<float>((p2.y - p1.y) / num_points * i + p1.y);
      point.z = static_cast<float>((p2.z - p1.z) / num_points * i + p1.z);
      cloud->points.push_back(point);
    }
    cloud->points.push_back(point2);
  }
}
double DegreeToRadian(double degree);
double RadianToDegree(double radian);
}

#endif  // MISC_UTILS_MISC_UTILS_H
