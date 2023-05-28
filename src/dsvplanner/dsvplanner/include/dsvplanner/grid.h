/**************************************************************************
grid.h
Header of the OccupancyGrid class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef GRID_H_
#define GRID_H_

#include "dsvplanner/drrt_base.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace Eigen;
namespace dsvplanner_ns
{
typedef Vector3d StateVec;
typedef Vector2i GridIndex;
class OccupancyGrid
{
  typedef std::shared_ptr<OccupancyGrid> Ptr;

public:
  OccupancyGrid(rclcpp::Node::SharedPtr& node_handle);
  ~OccupancyGrid();

  bool readParameters();
  bool initialize();

public:
  rclcpp::Node::SharedPtr nh_;

  // ROS subscribers
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> terrain_point_cloud_sub_;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  std::shared_ptr<Sync> sync_;
  // std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>> sync_;

  // ROS publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_cloud_pub_;

  // String constants
  std::string world_frame_id_;
  std::string sub_odom_topic_;
  std::string sub_terrain_point_cloud_topic_;
  std::string pub_grid_points_topic_;

  // Constants
  double kMapWidth;
  double kGridSize;
  double kDownsampleSize;
  double kObstacleHeightThre;
  double kFlyingObstacleHeightThre;
  double kCollisionCheckX;
  double kCollisionCheckY;

  // Variables
  enum gridStatus
  {
    unknown = 0,
    free = 1,
    occupied = 2,
    near_occupied = 3
  };
  std::vector<std::vector<int>> gridState_;
  int map_width_grid_num_;
  int map_half_width_grid_num_;

  rclcpp::Time terrain_time_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ds =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_traversable_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_obstacle_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  StateVec robot_position_;

  // Functions
  geometry_msgs::msg::Point getPoint(GridIndex p);
  GridIndex getIndex(StateVec point);
  void clearGrid();
  void updateGrid();
  void publishGridMap();
  bool collisionCheckByTerrainWithVector(StateVec origin_point, StateVec goal_point);
  bool collisionCheckByTerrain(geometry_msgs::msg::Point origin, geometry_msgs::msg::Point goal);
  bool InRange(const GridIndex sub, const GridIndex max_sub, const GridIndex min_sub);
  bool updateFreeGridWithSurroundingGrids(int indx, int indy);
  int signum(int x);
  double intbound(double s, double ds);
  double mod(double value, double modulus);
  std::vector<GridIndex> rayCast(GridIndex origin, GridIndex goal, GridIndex max_grid, GridIndex min_grid);

  // Callback Functions
  void terrainCloudAndOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                                   const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrain_msg);
};
}

#endif  // GRID_H
