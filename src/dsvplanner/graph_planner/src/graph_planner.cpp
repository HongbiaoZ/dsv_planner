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
#include <tf2/LinearMath/Matrix3x3.h>

#include "graph_planner.h"
#include <string>

namespace graph_planner_ns
{
bool GraphPlanner::readParameters()
{
  nh_->declare_parameter("world_frame_id", world_frame_id_);
  nh_->declare_parameter("graph_planner_command_topic", graph_planner_command_topic_);
  nh_->declare_parameter("graph_planner_status_topic", graph_planner_status_topic_);
  nh_->declare_parameter("pub_waypoint_topic", pub_waypoint_topic_);
  nh_->declare_parameter("pub_path_topic", pub_path_topic_);
  nh_->declare_parameter("sub_odometry_topic", sub_odometry_topic_);
  nh_->declare_parameter("sub_terrain_topic", sub_terrain_topic_);
  nh_->declare_parameter("sub_graph_topic", sub_graph_topic_);

  nh_->declare_parameter("kLookAheadDist", kLookAheadDist);
  nh_->declare_parameter("kWaypointProjectionDistance", kWaypointProjectionDistance);
  nh_->declare_parameter("kDownsampleSize", kDownsampleSize);
  nh_->declare_parameter("kObstacleHeightThres", kObstacleHeightThres);
  nh_->declare_parameter("kOverheadObstacleHeightThres", kOverheadObstacleHeightThres);
  nh_->declare_parameter("kCollisionCheckDistace", kCollisionCheckDistace);
  nh_->declare_parameter("kNextVertexMaintainTime", kNextVertexMaintainTime);
  nh_->declare_parameter("kExecuteFrequency", kExecuteFrequency);
  

  if (!nh_->get_parameter("world_frame_id", world_frame_id_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: world_frame_id_");
    return false;
  }
  if (!nh_->get_parameter("graph_planner_command_topic", graph_planner_command_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: graph_planner_command_topic_");
    return false;
  }
  if (!nh_->get_parameter("graph_planner_status_topic", graph_planner_status_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: graph_planner_status_topic_");
    return false;
  }
  if (!nh_->get_parameter("pub_waypoint_topic", pub_waypoint_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: pub_waypoint_topic_");
    return false;
  }
  if (!nh_->get_parameter("pub_path_topic", pub_path_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: pub_path_topic_");
    return false;
  }
  if (!nh_->get_parameter("sub_odometry_topic", sub_odometry_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: sub_odometry_topic_");
    return false;
  }
  if (!nh_->get_parameter("sub_terrain_topic", sub_terrain_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: sub_terrain_topic_");
    return false;
  }
  if (!nh_->get_parameter("sub_graph_topic", sub_graph_topic_))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: sub_graph_topic_");
    return false;
  }

  if (!nh_->get_parameter("kLookAheadDist", kLookAheadDist))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kLookAheadDist");
    return false;
  }
  if (!nh_->get_parameter("kWaypointProjectionDistance", kWaypointProjectionDistance))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kWaypointProjectionDistance");
    return false;
  }
  if (!nh_->get_parameter("kDownsampleSize", kDownsampleSize))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kDownsampleSize");
    return false;
  }
  if (!nh_->get_parameter("kObstacleHeightThres", kObstacleHeightThres))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kObstacleHeightThres");
    return false;
  }
  if (!nh_->get_parameter("kOverheadObstacleHeightThres", kOverheadObstacleHeightThres))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kOverheadObstacleHeightThres");
    return false;
  }
  if (!nh_->get_parameter("kCollisionCheckDistace", kCollisionCheckDistace))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kCollisionCheckDistace");
    return false;
  }
  if (!nh_->get_parameter("kNextVertexMaintainTime", kNextVertexMaintainTime))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kNextVertexMaintainTime");
    return false;
  }
  if (!nh_->get_parameter("kExecuteFrequency", kExecuteFrequency))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot read parameter: kExecuteFrequency");
    return false;
  }

  return true;
}

void GraphPlanner::publishPath()
{
  nav_msgs::msg::Path planned_path;
  planned_path.header.frame_id = world_frame_id_;
  planned_path.header.stamp = nh_->now();

  if (in_progress_ && !planned_path_.empty())
  {
    planned_path.poses.resize(planned_path_.size() + 1);
    planned_path.poses[0].pose.position = robot_pos_;
    for (int i = 0; i < planned_path_.size(); i++)
    {
      planned_path.poses[i + 1].pose.position = planned_path_[i];
    }
  }
  else
  {
    planned_path.poses.resize(1);
    planned_path.poses[0].pose.position = robot_pos_;
  }
  graph_planner_path_pub_->publish(planned_path);
}

void GraphPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat = odometry_msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

  robot_yaw_ = yaw;
  robot_pos_ = odometry_msg->pose.pose.position;
}

void GraphPlanner::commandCallback(const graph_planner::msg::GraphPlannerCommand::SharedPtr msg)
{
  graph_planner_command_ = *msg;
  if (graph_planner_command_.command == graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION)
  {
    previous_shortest_path_size_ = 100000;
  }
}

void GraphPlanner::graphCallback(const graph_utils::msg::TopologicalGraph::SharedPtr graph_msg)
{
  planned_graph_ = *graph_msg;
}

void GraphPlanner::terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr terrain_msg)
{
  terrain_point_->clear();
  terrain_point_crop_->clear();
  pcl::fromROSMsg(*terrain_msg, *terrain_point_);

  pcl::PointXYZI point;
  int terrainCloudSize = terrain_point_->points.size();
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point = terrain_point_->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    if (point.intensity > kObstacleHeightThres && point.intensity < kOverheadObstacleHeightThres)
    {
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

void GraphPlanner::publishInProgress(bool in_progress)
{
  in_progress_ = in_progress;  // used mainly for debugging

  graph_planner::msg::GraphPlannerStatus status;
  if (in_progress)
  {
    status.status = graph_planner::msg::GraphPlannerStatus::STATUS_IN_PROGRESS;
  }
  else
  {
    status.status = graph_planner::msg::GraphPlannerStatus::STATUS_DISABLED;
    state_ = ALL_OTHER_STATES;  // reset
  }

  graph_planner_status_pub_->publish(status);
}

void GraphPlanner::alterAndPublishWaypoint()
{
  // Ship it!
  waypoint_.header.frame_id = world_frame_id_;  // added by Hong in 20191203
  waypoint_.header.stamp = nh_->now();    // added by Hong in 20191203
  waypoint_pub_->publish(waypoint_);
}

bool GraphPlanner::goToVertex(int current_vertex_idx, int goal_vertex_idx)
{
  std::vector<int> shortest_path;
  graph_utils_ns::ShortestPathBtwVertex(shortest_path, planned_graph_, current_vertex_idx, goal_vertex_idx);

  if (shortest_path.size() <= 0)
  {
    // ROS_WARN("Graph planner did not find path to selected goal!");

    // clear the saved path
    planned_path_.clear();

    return false;
  }
  else if (shortest_path.size() <= 2)
  {
    geometry_msgs::msg::Point next_waypoint = planned_graph_.vertices[shortest_path.back()].location;

    waypoint_.header.stamp = nh_->now();
    waypoint_.point.x = next_waypoint.x;
    waypoint_.point.y = next_waypoint.y;
    waypoint_.point.z = next_waypoint.z;
    waypoint_.header.frame_id = world_frame_id_;

    // clear the saved path
    planned_path_.clear();
    planned_path_.push_back(waypoint_.point);

    alterAndPublishWaypoint();
    return false;
  }
  else
  {
    int next_vertex_id;
    next_vertex_id =
        graph_utils_ns::GetFirstVertexBeyondThreshold(robot_pos_, shortest_path, planned_graph_, kLookAheadDist);
    for (int i = 1; i < shortest_path.size(); i++)
    {
      //      std::cout << "i = " << i << " id is " << shortest_path[i] <<
      //      std::endl;
      if (shortest_path[i] != next_vertex_id && collisionCheckByTerrain(robot_pos_, shortest_path[i]))
      {
        next_vertex_id = shortest_path[i - 1];
        break;
      }
      else if (shortest_path[i] == next_vertex_id)
      {
        next_vertex_id = shortest_path[i];
        break;
      }
    }

    std::vector<int> next_shortest_path;
    graph_utils_ns::ShortestPathBtwVertex(next_shortest_path, planned_graph_, next_vertex_id, goal_vertex_idx);

    // prevent back and forth between two vertices
    if (next_shortest_path.size() > previous_shortest_path_size_)
    {
      next_vertex_id = previous_vertex_id_;
      backTraceCount_++;
      if (backTraceCount_ > kNextVertexMaintainTime * kExecuteFrequency)
      {
        backTraceCount_ = 0;
        previous_shortest_path_size_ = 100000;
      }
    }
    else
    {
      previous_vertex_id_ = next_vertex_id;
      previous_shortest_path_size_ = next_shortest_path.size();
    }

    geometry_msgs::msg::Point next_waypoint = planned_graph_.vertices[next_vertex_id].location;
    if (next_vertex_id != shortest_path.back())
    {
      next_waypoint = projectWayPoint(next_waypoint, robot_pos_);
    }
    waypoint_.header.stamp = nh_->now();
    waypoint_.point.x = next_waypoint.x;
    waypoint_.point.y = next_waypoint.y;
    waypoint_.point.z = next_waypoint.z;
    waypoint_.header.frame_id = world_frame_id_;

    // save path for debugging
    planned_path_.clear();
    for (const auto& v : shortest_path)
    {
      planned_path_.push_back(planned_graph_.vertices[v].location);
    }

    if (next_vertex_id == shortest_path.back())
    {
      // Assume reached vertex -- publish the last waypoint and then end
      alterAndPublishWaypoint();
      return false;
    }
    else
    {
      // Keep moving along planned path
      return true;
    }
  }
}

bool GraphPlanner::goToPoint(geometry_msgs::msg::Point point)
{
  if (planned_graph_.vertices.size() <= 0)
  {
    return false;
  }
  // Find where I am on this graph
  int current_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(planned_graph_, robot_pos_);

  // find closest goal vertex
  int goal_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(planned_graph_, point);

  // Set the waypoint
  return goToVertex(current_vertex_idx, goal_vertex_idx);
}

bool GraphPlanner::collisionCheckByTerrain(geometry_msgs::msg::Point robot_position, int end_vertex_idx)
{
  geometry_msgs::msg::Point start_point = robot_position;
  geometry_msgs::msg::Point end_point = planned_graph_.vertices[end_vertex_idx].location;

  int count = 0;
  double distance = sqrt((end_point.x - start_point.x) * (end_point.x - start_point.x) +
                         (end_point.y - start_point.y) * (end_point.y - start_point.y));
  double check_point_num = distance / (kCollisionCheckDistace);
  for (int j = 0; j < check_point_num; j++)
  {
    geometry_msgs::msg::Point p1;
    p1.x = start_point.x + j * kCollisionCheckDistace / distance * (end_point.x - start_point.x);
    p1.y = start_point.y + j * kCollisionCheckDistace / distance * (end_point.y - start_point.y);
    for (int i = 0; i < terrain_point_crop_->points.size(); i++)
    {
      double dist = sqrt((p1.x - terrain_point_crop_->points[i].x) * (p1.x - terrain_point_crop_->points[i].x) +
                         (p1.y - terrain_point_crop_->points[i].y) * (p1.y - terrain_point_crop_->points[i].y));
      if (dist < kCollisionCheckDistace)
      {
        count++;
      }
      if (count > 0)
      {
        return true;
      }
    }
  }
  return false;
}
geometry_msgs::msg::Point GraphPlanner::projectWayPoint(geometry_msgs::msg::Point next_vertex_pos, geometry_msgs::msg::Point robot_pos)
{
  double ratio = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(robot_pos, next_vertex_pos) /
                 kWaypointProjectionDistance;
  double x = (next_vertex_pos.x - robot_pos.x) / ratio + robot_pos.x;
  double y = (next_vertex_pos.y - robot_pos.y) / ratio + robot_pos.y;
  double z = (next_vertex_pos.z - robot_pos.z) / ratio + robot_pos.z;
  geometry_msgs::msg::Point way_point = misc_utils_ns::GeoMsgPoint(x, y, z);
  return way_point;
}

bool GraphPlanner::initialize()
{
  if (!readParameters())
    return false;

  // Initialize target waypoint
  waypoint_ = geometry_msgs::msg::PointStamped();
  waypoint_.point.x = -1.0;
  waypoint_.point.y = -1.0;
  waypoint_.point.z = 0.0;
  waypoint_.header.frame_id = world_frame_id_;

  // Initialize subscribers
  graph_sub_ = nh_->create_subscription<graph_utils::msg::TopologicalGraph>(sub_graph_topic_, 5, 
  std::bind(&GraphPlanner::graphCallback, this, std::placeholders::_1));

  graph_planner_command_sub_ = nh_->create_subscription<graph_planner::msg::GraphPlannerCommand>(graph_planner_command_topic_, 5, 
  std::bind(&GraphPlanner::commandCallback, this, std::placeholders::_1));

  odometry_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(sub_odometry_topic_, 5, 
  std::bind(&GraphPlanner::odometryCallback, this, std::placeholders::_1));

  terrain_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(sub_terrain_topic_, 5, 
  std::bind(&GraphPlanner::terrainCallback, this, std::placeholders::_1));

  // Initialize publishers
  waypoint_pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(pub_waypoint_topic_, 10);
  graph_planner_status_pub_ = nh_->create_publisher<graph_planner::msg::GraphPlannerStatus>(graph_planner_status_topic_, 10);
  graph_planner_path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>(pub_path_topic_, 10);

  RCLCPP_INFO(nh_->get_logger(), "Successfully launched graph_planner node!");

  return true;
}

void GraphPlanner::executeGoToOrigin()
{
  // Aim for (0,0,0)
  geometry_msgs::msg::Point origin;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  bool success = goToPoint(origin);

  if (!success)
  {
    publishInProgress(false);
  }
  else
  {
    alterAndPublishWaypoint();
    publishInProgress(true);
  }
}

void GraphPlanner::executeGoToLocation()
{
  // Compute waypoint
  bool success = goToPoint(graph_planner_command_.location);

  if (!success)
  {
    publishInProgress(false);
  }
  else
  {
    alterAndPublishWaypoint();
    publishInProgress(true);
  }
}

void GraphPlanner::executeCommand()
{
  // Choose from one of the graph_planner variants based on the command

  if (graph_planner_command_.command == graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_ORIGIN)
  {
    // COMMAND_GO_TO_ORIGIN
    executeGoToOrigin();
  }
  else if (graph_planner_command_.command == graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION)
  {
    // COMMAND_GO_TO_LOCATION
    executeGoToLocation();
  }
  else
  {
    publishInProgress(false);
  }
}

bool GraphPlanner::execute()
{
  rclcpp::Rate loopRate(kExecuteFrequency);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);

    if (graph_planner_command_.command == graph_planner::msg::GraphPlannerCommand::COMMAND_DISABLE)
    {
      // Graph planner is disabled -- do nothing
      publishInProgress(false);
    }
    else
    {
      // Graph planner is enabled -- select waypoint according to current
      // command
      executeCommand();
    }

    // Generate and publish the path
    publishPath();

    loopRate.sleep();
  }

  return true;
}

GraphPlanner::GraphPlanner(rclcpp::Node::SharedPtr& node_handle)
{
  nh_ = node_handle;
  initialize();
}

}  // namespace graph_planner_ns
