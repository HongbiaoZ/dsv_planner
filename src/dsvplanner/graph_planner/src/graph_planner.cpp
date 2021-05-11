/**************************************************************************
graph_planner.cpp
Implementation of GraphPlanner class
Subscribes to local graph and search path to goal point on this graph

Created by Chao Cao (ccao1@andrew.cmu.edu)
6/3/19

Modified and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include <graph_utils.h>
#include <misc_utils/misc_utils.h>

#include "graph_planner.h"
#include <tf/transform_datatypes.h>

#include <string>

namespace graph_planner_ns {
bool GraphPlanner::readParameters() {
  if (!nh_.getParam("world_frame_id", world_frame_id_)) {
    ROS_ERROR("Cannot read parameter: world_frame_id_");
    return false;
  }
  if (!nh_.getParam("graph_planner_command_topic",
                    graph_planner_command_topic_)) {
    ROS_ERROR("Cannot read parameter: graph_planner_command_topic_");
    return false;
  }
  if (!nh_.getParam("graph_planner_status_topic",
                    graph_planner_status_topic_)) {
    ROS_ERROR("Cannot read parameter: graph_planner_status_topic_");
    return false;
  }
  if (!nh_.getParam("pub_waypoint_topic", pub_waypoint_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_waypoint_topic_");
    return false;
  }
  if (!nh_.getParam("pub_path_topic", pub_path_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_path_topic_");
    return false;
  }
  if (!nh_.getParam("sub_odometry_topic", sub_odometry_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_odometry_topic_");
    return false;
  }
  if (!nh_.getParam("sub_terrain_topic", sub_terrain_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_terrain_topic_");
    return false;
  }
  if (!nh_.getParam("sub_graph_topic", sub_graph_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_graph_topic_");
    return false;
  }

  if (!nh_.getParam("kLookAheadDist", kLookAheadDist)) {
    ROS_ERROR("Cannot read parameter: kLookAheadDist");
    return false;
  }
  if (!nh_.getParam("kWaypointProjectionDistance",
                    kWaypointProjectionDistance)) {
    ROS_ERROR("Cannot read parameter: kWaypointProjectionDistance");
    return false;
  }
  if (!nh_.getParam("kDownsampleSize", kDownsampleSize)) {
    ROS_ERROR("Cannot read parameter: kDownsampleSize");
    return false;
  }
  if (!nh_.getParam("kObstacleHeightThres", kObstacleHeightThres)) {
    ROS_ERROR("Cannot read parameter: kObstacleHeightThres");
    return false;
  }
  if (!nh_.getParam("kOverheadObstacleHeightThres",
                    kOverheadObstacleHeightThres)) {
    ROS_ERROR("Cannot read parameter: kOverheadObstacleHeightThres");
    return false;
  }
  if (!nh_.getParam("kCollisionCheckDistace", kCollisionCheckDistace)) {
    ROS_ERROR("Cannot read parameter: kCollisionCheckDistace");
    return false;
  }

  return true;
}

void GraphPlanner::publishPath() {
  nav_msgs::Path planned_path;
  planned_path.header.frame_id = world_frame_id_;
  planned_path.header.stamp = ros::Time::now();

  if (in_progress_ && !planned_path_.empty()) {
    planned_path.poses.resize(planned_path_.size() + 1);
    planned_path.poses[0].pose.position = robot_pos_;
    for (int i = 0; i < planned_path_.size(); i++) {
      planned_path.poses[i + 1].pose.position = planned_path_[i];
    }
  } else {
    planned_path.poses.resize(1);
    planned_path.poses[0].pose.position = robot_pos_;
  }
  graph_planner_path_pub_.publish(planned_path);
}

void GraphPlanner::odometryCallback(
    const nav_msgs::Odometry::ConstPtr &odometry_msg) {
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat = odometry_msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w))
      .getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;
  robot_pos_ = odometry_msg->pose.pose.position;
}

void GraphPlanner::commandCallback(
    const graph_planner::GraphPlannerCommand::ConstPtr &msg) {
  graph_planner_command_ = *msg;
  if (graph_planner_command_.command ==
      graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION) {
    previous_shortest_path_size_ = 100000;
    wrong_id_shortest_path_size_ = 100000;
    previous_shortest_path_size_when_pathrewind = 100000;
  }
}

void GraphPlanner::graphCallback(
    const graph_utils::TopologicalGraph::ConstPtr &graph_msg) {
  planned_graph_ = *graph_msg;
}

void GraphPlanner::terrainCallback(
    const sensor_msgs::PointCloud2::ConstPtr &terrain_msg) {
  terrain_point_->clear();
  terrain_point_crop_->clear();
  pcl::fromROSMsg(*terrain_msg, *terrain_point_);

  pcl::PointXYZI point;
  int terrainCloudSize = terrain_point_->points.size();
  for (int i = 0; i < terrainCloudSize; i++) {
    point = terrain_point_->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    if (point.intensity > kObstacleHeightThres &&
        point.intensity < kOverheadObstacleHeightThres) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      terrain_point_crop_->push_back(point);
    }
  }
  pcl::VoxelGrid<pcl::PointXYZI> point_ds;
  point_ds.setLeafSize(kDownsampleSize, kDownsampleSize, kDownsampleSize);
  point_ds.setInputCloud(terrain_point_crop_);
  point_ds.filter(*terrain_point_crop_);
}

void GraphPlanner::publishInProgress(bool in_progress) {
  in_progress_ = in_progress; // used mainly for debugging

  graph_planner::GraphPlannerStatus status;
  if (in_progress) {
    status.status = graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS;
  } else {
    status.status = graph_planner::GraphPlannerStatus::STATUS_DISABLED;
    state_ = ALL_OTHER_STATES; // reset
  }

  graph_planner_status_pub_.publish(status);
}

void GraphPlanner::alterAndPublishWaypoint() {
  // Ship it!
  waypoint_.header.frame_id = world_frame_id_; // added by Hong in 20191203
  waypoint_.header.stamp = ros::Time::now();   // added by Hong in 20191203
  waypoint_pub_.publish(waypoint_);
}

bool GraphPlanner::goToVertex(int current_vertex_idx, int goal_vertex_idx) {
  std::vector<int> shortest_path;
  graph_utils_ns::ShortestPathBtwVertex(shortest_path, planned_graph_,
                                        current_vertex_idx, goal_vertex_idx);

  if (shortest_path.size() <= 0) {
    // ROS_WARN("Graph planner did not find path to selected goal!");

    // clear the saved path
    planned_path_.clear();

    return false;
  } else if (shortest_path.size() <= 2) {
    geometry_msgs::Point next_waypoint =
        planned_graph_.vertices[shortest_path.back()].location;

    waypoint_.header.stamp = ros::Time::now();
    waypoint_.point.x = next_waypoint.x;
    waypoint_.point.y = next_waypoint.y;
    waypoint_.point.z = next_waypoint.z;
    waypoint_.header.frame_id = world_frame_id_;

    // clear the saved path
    planned_path_.clear();
    planned_path_.push_back(waypoint_.point);

    alterAndPublishWaypoint();
    return false;
  } else {
    int next_vertex_id;
    next_vertex_id = graph_utils_ns::GetFirstVertexBeyondThreshold(
        robot_pos_, shortest_path, planned_graph_, kLookAheadDist);
    for (int i = 1; i < shortest_path.size(); i++) {
      //      std::cout << "i = " << i << " id is " << shortest_path[i] <<
      //      std::endl;
      if (shortest_path[i] != next_vertex_id &&
          collisionCheckByTerrain(robot_pos_, shortest_path[i])) {
        next_vertex_id = shortest_path[i - 1];
        break;
      } else if (shortest_path[i] == next_vertex_id) {
        next_vertex_id = shortest_path[i];
        break;
      }
    }

    std::vector<int> next_shortest_path;
    graph_utils_ns::ShortestPathBtwVertex(next_shortest_path, planned_graph_,
                                          next_vertex_id, goal_vertex_idx);
    //    std::vector<int> next_shortest_path;
    //    graph_utils_ns::ShortestPathBtwVertex(next_shortest_path,
    //    planned_graph_,
    //                                          next_vertex_id,
    //                                          goal_vertex_idx);

    //    // when the path is bypassed a thin wall, follow the path one vertex
    //    by one
    //    // vertex.
    //    bool pathRewind = graph_utils_ns::PathCircleDetect(
    //        shortest_path, planned_graph_, next_vertex_id, robot_pos_);
    //    bool collisionCheckResult =
    //        collisionCheckByTerrain(robot_pos_, next_vertex_id);
    //    if (pathRewind && collisionCheckResult) {
    //      // std::cout << "pathrewind = " << pathRewind << std::endl;
    //      wrong_id_ = true;
    //      wrong_id_shortest_path_size_ = next_shortest_path.size();
    //    }
    // prevent back and forth between two vertices
    if (next_shortest_path.size() > previous_shortest_path_size_) {
      next_vertex_id = previous_vertex_id_;
      backTraceCount_++;
      if (backTraceCount_ > 15) {
        backTraceCount_ = 0;
        previous_shortest_path_size_ = 100000;
      }
    } else {
      previous_vertex_id_ = next_vertex_id;
      previous_shortest_path_size_ = next_shortest_path.size();
    }

    geometry_msgs::Point next_waypoint =
        planned_graph_.vertices[next_vertex_id].location;
    if (next_vertex_id != shortest_path.back()) {
      next_waypoint = projectWayPoint(next_waypoint, robot_pos_);
    }
    waypoint_.header.stamp = ros::Time::now();
    waypoint_.point.x = next_waypoint.x;
    waypoint_.point.y = next_waypoint.y;
    waypoint_.point.z = next_waypoint.z;
    waypoint_.header.frame_id = world_frame_id_;

    // save path for debugging
    planned_path_.clear();
    for (const auto &v : shortest_path) {
      planned_path_.push_back(planned_graph_.vertices[v].location);
    }

    if (next_vertex_id == shortest_path.back()) {
      // Assume reached vertex -- publish the last waypoint and then end
      alterAndPublishWaypoint();
      return false;
    } else {
      // Keep moving along planned path
      return true;
    }
  }
}

bool GraphPlanner::goToPoint(geometry_msgs::Point point) {
  if (planned_graph_.vertices.size() <= 0) {
    return false;
  }
  // Find where I am on this graph
  int current_vertex_idx =
      graph_utils_ns::GetClosestVertexIdxToPoint(planned_graph_, robot_pos_);

  // find closest goal vertex
  int goal_vertex_idx =
      graph_utils_ns::GetClosestVertexIdxToPoint(planned_graph_, point);

  // Set the waypoint
  return goToVertex(current_vertex_idx, goal_vertex_idx);
}

bool GraphPlanner::collisionCheckByTerrain(geometry_msgs::Point robot_position,
                                           int end_vertex_idx) {
  geometry_msgs::Point start_point = robot_position;
  geometry_msgs::Point end_point =
      planned_graph_.vertices[end_vertex_idx].location;

  int count = 0;
  double distance =
      sqrt((end_point.x - start_point.x) * (end_point.x - start_point.x) +
           (end_point.y - start_point.y) * (end_point.y - start_point.y));
  double check_point_num = distance / (kCollisionCheckDistace);
  for (int j = 0; j < check_point_num; j++) {
    geometry_msgs::Point p1;
    p1.x =
        start_point.x +
        j * kCollisionCheckDistace / distance * (end_point.x - start_point.x);
    p1.y =
        start_point.y +
        j * kCollisionCheckDistace / distance * (end_point.y - start_point.y);
    for (int i = 0; i < terrain_point_crop_->points.size(); i++) {
      double dist = sqrt((p1.x - terrain_point_crop_->points[i].x) *
                             (p1.x - terrain_point_crop_->points[i].x) +
                         (p1.y - terrain_point_crop_->points[i].y) *
                             (p1.y - terrain_point_crop_->points[i].y));
      if (dist < kCollisionCheckDistace) {
        count++;
      }
      if (count > 0) {
        return true;
      }
    }
  }
  return false;
}
geometry_msgs::Point
GraphPlanner::projectWayPoint(geometry_msgs::Point next_vertex_pos,
                              geometry_msgs::Point robot_pos) {
  double ratio =
      misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
          robot_pos, next_vertex_pos) /
      kWaypointProjectionDistance;
  double x = (next_vertex_pos.x - robot_pos.x) / ratio + robot_pos.x;
  double y = (next_vertex_pos.y - robot_pos.y) / ratio + robot_pos.y;
  double z = (next_vertex_pos.z - robot_pos.z) / ratio + robot_pos.z;
  geometry_msgs::Point way_point = misc_utils_ns::GeoMsgPoint(x, y, z);
  return way_point;
}

bool GraphPlanner::initialize() {
  if (!readParameters())
    return false;

  // Initialize target waypoint
  waypoint_ = geometry_msgs::PointStamped();
  waypoint_.point.x = -1.0;
  waypoint_.point.y = -1.0;
  waypoint_.point.z = 0.0;
  waypoint_.header.frame_id = world_frame_id_;

  // Initialize subscribers
  graph_sub_ =
      nh_.subscribe(sub_graph_topic_, 1, &GraphPlanner::graphCallback, this);
  graph_planner_command_sub_ = nh_.subscribe(
      graph_planner_command_topic_, 1, &GraphPlanner::commandCallback, this);
  odometry_sub_ = nh_.subscribe(sub_odometry_topic_, 1,
                                &GraphPlanner::odometryCallback, this);
  terrain_sub_ = nh_.subscribe(sub_terrain_topic_, 1,
                               &GraphPlanner::terrainCallback, this);

  // Initialize publishers
  waypoint_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>(pub_waypoint_topic_, 2);
  graph_planner_status_pub_ = nh_.advertise<graph_planner::GraphPlannerStatus>(
      graph_planner_status_topic_, 2);
  graph_planner_path_pub_ = nh_.advertise<nav_msgs::Path>(pub_path_topic_, 10);

  ROS_INFO("Successfully launched graph_planner node");

  return true;
}

void GraphPlanner::executeGoToOrigin() {
  // Aim for (0,0,0)
  geometry_msgs::Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  bool success = goToPoint(origin);

  if (!success) {
    publishInProgress(false);
  } else {
    alterAndPublishWaypoint();
    publishInProgress(true);
  }
}

void GraphPlanner::executeGoToLocation() {
  // Compute waypoint
  bool success = goToPoint(graph_planner_command_.location);

  if (!success) {
    publishInProgress(false);
  } else {
    alterAndPublishWaypoint();
    publishInProgress(true);
  }
}

void GraphPlanner::executeCommand() {
  // Choose from one of the graph_planner variants based on the command

  if (graph_planner_command_.command ==
      graph_planner::GraphPlannerCommand::COMMAND_GO_TO_ORIGIN) {
    // COMMAND_GO_TO_ORIGIN
    executeGoToOrigin();
  } else if (graph_planner_command_.command ==
             graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION) {
    // COMMAND_GO_TO_LOCATION
    executeGoToLocation();
  } else {
    publishInProgress(false);
  }
}

bool GraphPlanner::execute() {
  ros::Rate rate(5);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (graph_planner_command_.command ==
        graph_planner::GraphPlannerCommand::COMMAND_DISABLE) {
      // Graph planner is disabled -- do nothing
      publishInProgress(false);
    } else {
      // Graph planner is enabled -- select waypoint according to current
      // command
      executeCommand();
    }

    // Generate and publish the path
    publishPath();

    status = ros::ok();
    rate.sleep();
  }

  return true;
}

GraphPlanner::GraphPlanner(const ros::NodeHandle &nh) {
  nh_ = nh;
  initialize();
}

} // namespace graph_planner_ns
