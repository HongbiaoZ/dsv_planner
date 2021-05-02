/*
grid.cpp
Implementation of OccupancyGrid class. Occupancy grid is used to get do the
collision check
based on terrain points in
local area.

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#ifndef GRID_HPP_
#define GRID_HPP_

#include "dsvplanner/grid.h"

namespace dsvplanner_ns {
OccupancyGrid::OccupancyGrid(const ros::NodeHandle &nh,
                             const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  initialize();
}

OccupancyGrid::~OccupancyGrid() {}

bool OccupancyGrid::readParameters() {
  nh_private_.getParam("/grid/world_frame_id", world_frame_id_);
  nh_private_.getParam("/grid/odomSubTopic", sub_odom_topic_);
  nh_private_.getParam("/grid/terrainCloudSubTopic",
                       sub_terrain_point_cloud_topic_);
  nh_private_.getParam("/grid/pubGridPointsTopic", pub_grid_points_topic_);
  nh_private_.getParam("/grid/kMapWidth", kMapWidth);
  nh_private_.getParam("/grid/kGridSize", kGridSize);
  nh_private_.getParam("/grid/kDownsampleSize", kDownsampleSize);
  nh_private_.getParam("/grid/kObstacleHeightThre", kObstacleHeightThre);
  nh_private_.getParam("/grid/kFlyingObstacleHeightThre",
                       kFlyingObstacleHeightThre);

  return true;
}

bool OccupancyGrid::initialize() {
  // Read in parameters
  if (!readParameters())
    return false;
  // Initialize subscriber
  odom_sub_.subscribe(nh_, sub_odom_topic_, 1);
  terrain_point_cloud_sub_.subscribe(nh_, sub_terrain_point_cloud_topic_, 1);
  sync_.reset(new Sync(syncPolicy(10), odom_sub_, terrain_point_cloud_sub_));
  sync_->registerCallback(
      boost::bind(&OccupancyGrid::terrainCloudAndOdomCallback, this, _1, _2));

  grid_cloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(pub_grid_points_topic_, 1);

  map_half_width_grid_num_ = int(kMapWidth / 2 / kGridSize);
  map_width_grid_num_ = map_half_width_grid_num_ * 2 + 1;

  clearGrid();

  ROS_INFO("Successfully launched OccupancyGrid node");

  return true;
}

void OccupancyGrid::terrainCloudAndOdomCallback(
    const nav_msgs::Odometry::ConstPtr &odom_msg,
    const sensor_msgs::PointCloud2::ConstPtr &terrain_msg) {
  terrain_time_ = terrain_msg->header.stamp;
  robot_position_[0] = odom_msg->pose.pose.position.x;
  robot_position_[1] = odom_msg->pose.pose.position.y;
  robot_position_[2] = odom_msg->pose.pose.position.z;
  terrain_cloud_->clear();
  terrain_cloud_ds->clear();
  terrain_cloud_traversable_->clear();
  terrain_cloud_obstacle_->clear();
  pcl::fromROSMsg(*terrain_msg, *terrain_cloud_);

  pcl::VoxelGrid<pcl::PointXYZI> point_ds;
  point_ds.setLeafSize(kDownsampleSize, kDownsampleSize, kDownsampleSize);
  point_ds.setInputCloud(terrain_cloud_);
  point_ds.filter(*terrain_cloud_ds);

  pcl::PointXYZI point;
  int terrainCloudSize = terrain_cloud_ds->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point.x = terrain_cloud_ds->points[i].x;
    point.y = terrain_cloud_ds->points[i].y;
    point.z = terrain_cloud_ds->points[i].z;
    point.intensity = terrain_cloud_ds->points[i].intensity;
    // crop all ground points
    if (point.intensity > kObstacleHeightThre &&
        point.intensity < kFlyingObstacleHeightThre) {
      terrain_cloud_obstacle_->push_back(point);
    } else if (point.intensity <= kObstacleHeightThre) {
      terrain_cloud_traversable_->push_back(point);
    }
  }

  clearGrid();
  updateGrid();
  publishGridMap();
}

geometry_msgs::Point OccupancyGrid::getPoint(GridIndex p) {
  int indX = p[0];
  int indY = p[1];
  double x = kGridSize * (indX - map_half_width_grid_num_) + robot_position_[0];
  double y = kGridSize * (indY - map_half_width_grid_num_) + robot_position_[1];
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = robot_position_[2];
  return point;
}

GridIndex OccupancyGrid::getIndex(StateVec point) {
  int indX = int((point.x() - robot_position_[0] + kGridSize / 2) / kGridSize) +
             map_half_width_grid_num_;
  int indY = int((point.y() - robot_position_[1] + kGridSize / 2) / kGridSize) +
             map_half_width_grid_num_;
  if (point.x() - robot_position_[0] + kGridSize / 2 < 0)
    indX--;
  if (point.y() - robot_position_[1] + kGridSize / 2 < 0)
    indY--;
  if (indX < 0)
    indX = 0;
  if (indY < 0)
    indY = 0;
  if (indX > map_width_grid_num_ - 1)
    indX = map_width_grid_num_ - 1;
  if (indY > map_width_grid_num_ - 1)
    indY = map_width_grid_num_ - 1;
  GridIndex grid_index(indX, indY);
  return grid_index;
}

void OccupancyGrid::clearGrid() {
  gridState_.clear();
  std::vector<int> y_vector;
  for (int i = 0; i < map_width_grid_num_; i++) {
    y_vector.clear();
    for (int j = 0; j < map_width_grid_num_; j++) {
      gridStatus grid_state = unknown;
      y_vector.push_back(grid_state);
    }
    gridState_.push_back(y_vector);
  }
}

void OccupancyGrid::updateGrid() {
  pcl::PointXYZI point;
  for (int i = 0; i < terrain_cloud_traversable_->points.size(); i++) {
    point = terrain_cloud_traversable_->points[i];
    int indX = int((point.x - robot_position_[0] + kGridSize / 2) / kGridSize) +
               map_half_width_grid_num_;
    int indY = int((point.y - robot_position_[1] + kGridSize / 2) / kGridSize) +
               map_half_width_grid_num_;
    if (point.x - robot_position_[0] + kGridSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kGridSize / 2 < 0)
      indY--;
    if (indX < 0)
      indX = 0;
    if (indY < 0)
      indY = 0;
    if (indX > map_width_grid_num_ - 1)
      indX = map_width_grid_num_ - 1;
    if (indY > map_width_grid_num_ - 1)
      indY = map_width_grid_num_ - 1;
    if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 &&
        indY < map_width_grid_num_) {
      gridStatus grid_state = free;
      gridState_[indX][indY] = grid_state;
    }
  }
  for (int i = 0; i < terrain_cloud_obstacle_->points.size(); i++) {
    point = terrain_cloud_obstacle_->points[i];
    int indX = int((point.x - robot_position_[0] + kGridSize / 2) / kGridSize) +
               map_half_width_grid_num_;
    int indY = int((point.y - robot_position_[1] + kGridSize / 2) / kGridSize) +
               map_half_width_grid_num_;
    if (point.x - robot_position_[0] + kGridSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kGridSize / 2 < 0)
      indY--;
    if (indX < 0)
      indX = 0;
    if (indY < 0)
      indY = 0;
    if (indX > map_width_grid_num_ - 1)
      indX = map_width_grid_num_ - 1;
    if (indY > map_width_grid_num_ - 1)
      indY = map_width_grid_num_ - 1;

    if (indX >= 0 && indX < map_width_grid_num_ && indY >= 0 &&
        indY < map_width_grid_num_) {
      gridStatus grid_state = occupied;
      gridState_[indX][indY] = grid_state;
    }
  }
}

void OccupancyGrid::publishGridMap() {
  grid_cloud_->clear();
  pcl::PointXYZI p1;
  geometry_msgs::Point p2;
  GridIndex p3;
  for (int i = 0; i < map_width_grid_num_; i++) {
    for (int j = 0; j < map_width_grid_num_; j++) {
      p3[0] = i;
      p3[1] = j;
      p2 = getPoint(p3);
      p1.x = p2.x;
      p1.y = p2.y;
      p1.z = p2.z;
      p1.intensity = gridState_[i][j];
      grid_cloud_->points.push_back(p1);
    }
  }
  sensor_msgs::PointCloud2 gridCloud2;
  pcl::toROSMsg(*grid_cloud_, gridCloud2);
  gridCloud2.header.stamp = terrain_time_;
  gridCloud2.header.frame_id = world_frame_id_;
  grid_cloud_pub_.publish(gridCloud2);
}

bool OccupancyGrid::collisionCheckByTerrainWithVector(StateVec origin_point,
                                                      StateVec goal_point) {
  //  ROS_INFO("Start Check Collision");
  GridIndex origin_grid_index = getIndex(origin_point);
  GridIndex goal_grid_index = getIndex(goal_point);
  GridIndex max_grid_index(map_width_grid_num_ - 1, map_width_grid_num_ - 1);
  GridIndex min_grid_index(0, 0);
  GridIndex grid_index;
  std::vector<GridIndex> ray_tracing_grids = rayCast(
      origin_grid_index, goal_grid_index, max_grid_index, min_grid_index);
  int length = ray_tracing_grids.size();
  for (int i = 0; i < length; i++) {
    grid_index = ray_tracing_grids[i];
    if (gridState_[grid_index[0]][grid_index[1]] == 2) {
      //      ROS_INFO("Successfully Check Collision");
      return true;
    }
  }
  //  ROS_INFO("Successfully Check Collision");
  return false;
}

bool OccupancyGrid::collisionCheckByTerrain(geometry_msgs::Point origin,
                                            geometry_msgs::Point goal) {
  StateVec origin_point(origin.x, origin.y, origin.z);
  StateVec goal_point(goal.x, goal.y, goal.z);

  return collisionCheckByTerrainWithVector(origin_point, goal_point);
}

bool OccupancyGrid::InRange(const GridIndex sub, const GridIndex max_sub,
                            const GridIndex min_sub) {
  return sub.x() >= min_sub.x() && sub.x() <= max_sub.x() &&
         sub.y() >= min_sub.y() && sub.y() <= max_sub.y();
}

int OccupancyGrid::signum(int x) { return x == 0 ? 0 : x < 0 ? -1 : 1; }

double OccupancyGrid::intbound(double s, double ds) {
  // Find the smallest positive t such that s+t*ds is an integer.
  if (ds < 0) {
    return intbound(-s, -ds);
  } else {
    s = mod(s, 1);
    // problem is now s+t*ds = 1
    return (1 - s) / ds;
  }
}

double OccupancyGrid::mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

std::vector<GridIndex> OccupancyGrid::rayCast(GridIndex origin, GridIndex goal,
                                              GridIndex max_grid,
                                              GridIndex min_grid) {
  std::vector<GridIndex> grid_pairs;
  if (origin == goal) {
    grid_pairs.push_back(origin);
    return grid_pairs;
  }
  GridIndex diff = goal - origin;
  double max_dist = diff.squaredNorm();
  int step_x = signum(diff.x());
  int step_y = signum(diff.y());
  double t_max_x = step_x == 0 ? DBL_MAX : intbound(origin.x(), diff.x());
  double t_max_y = step_y == 0 ? DBL_MAX : intbound(origin.y(), diff.y());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff.y();
  double dist = 0;
  GridIndex cur_sub = origin;

  while (true) {
    if (InRange(cur_sub, max_grid, min_grid)) {
      grid_pairs.push_back(cur_sub);
      dist = (cur_sub - origin).squaredNorm();
      if (cur_sub == goal || dist > max_dist) {
        return grid_pairs;
      }
      if (t_max_x < t_max_y) {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
      } else {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
      }
    } else {
      return grid_pairs;
    }
  }
}
}
#endif
