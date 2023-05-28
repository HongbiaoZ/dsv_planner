/**************************************************************************
graph_planner.h
Header of the graph_planner class

Created by Chao Cao (ccao1@andrew.cmu.edu)
6/3/19

Modified and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#ifndef GRAPH_PLANNER_H
#define GRAPH_PLANNER_H
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs//msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "graph_planner/msg/graph_planner_command.hpp"
#include "graph_planner/msg/graph_planner_status.hpp"
#include "graph_utils/msg/edge.hpp"
#include "graph_utils/msg/topological_graph.hpp"
#include "graph_utils/msg/vertex.hpp"

namespace graph_planner_ns
{
typedef pcl::PointXYZ PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLCloudType;

class GraphPlanner;
typedef struct
{
  bool in_polygon;
  float cost_estimate;
} VertexScore;
}  // namespace dfs_behavior_planner_ns

class graph_planner_ns::GraphPlanner
{
private:
  // ROS Nodehandler
  rclcpp::Node::SharedPtr nh_;

  // ROS subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Subscription<graph_utils::msg::TopologicalGraph>::SharedPtr graph_sub_;
  rclcpp::Subscription<graph_planner::msg::GraphPlannerCommand>::SharedPtr graph_planner_command_sub_;

  // ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<graph_planner::msg::GraphPlannerStatus>::SharedPtr graph_planner_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr graph_planner_path_pub_;

  // String constants
  std::string world_frame_id_;
  std::string sub_odometry_topic_;
  std::string sub_terrain_topic_;
  std::string sub_graph_topic_;
  std::string graph_planner_command_topic_;
  std::string graph_planner_status_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_path_topic_;

  // Constants
  float kLookAheadDist;
  double kWaypointProjectionDistance;
  double kDownsampleSize;
  double kObstacleHeightThres;
  double kOverheadObstacleHeightThres;
  double kCollisionCheckDistace;
  double kNextVertexMaintainTime;
  int kExecuteFrequency;

  // "State" variables
  enum GraphPlannerState
  {
    ALL_OTHER_STATES = 0,
    NEARBY_OPENING = 1
  };
  GraphPlannerState state_;

  // Variables
  graph_planner::msg::GraphPlannerCommand graph_planner_command_;  // received command
  graph_utils::msg::TopologicalGraph planned_graph_;               // the received graph that can be palnned path in
  geometry_msgs::msg::PointStamped waypoint_;                      // goal waypoint being published by
                                                              // this node (if in mode IN_CONTROL)
  geometry_msgs::msg::Point robot_pos_;                            // current robot position
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_point_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_point_crop_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<geometry_msgs::msg::Point> planned_path_;  // only used for debugging --
                                                    // the waypoint_ is what is
                                                    // actually
                                                    // output
  double robot_yaw_;                                // current robot yaw
  bool in_progress_ = false;                        // current graph planner status
  bool wrong_id_ = false;
  int backTraceCount_ = 0;
  int previous_shortest_path_size_ = 100000;  // the size of the previous planned
                                              // path. used to avoid moving back
                                              // and forth in some cases
  int previous_vertex_id_;                    // the id of the previous planned goal vertex

  // Callbacks
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);
  void graphCallback(const graph_utils::msg::TopologicalGraph::SharedPtr graph_msg);
  void commandCallback(const graph_planner::msg::GraphPlannerCommand::SharedPtr msg);
  void terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr terrain_msg);
  // Other functions
  void publishPath();
  void alterAndPublishWaypoint();
  void publishInProgress(bool in_progress);
  bool readParameters();
  bool goToVertex(int current_vertex_idx, int goal_vertex_idx);
  bool goToPoint(geometry_msgs::msg::Point point);
  bool collisionCheckByTerrain(geometry_msgs::msg::Point robot_position, int end_vertex_idx);
  geometry_msgs::msg::Point projectWayPoint(geometry_msgs::msg::Point next_vertex_pos, geometry_msgs::msg::Point robot_pos);

  // Execute function variants
  void executeGoToOrigin();    // for returning home
  void executeGoToLocation();  // for going to goal point
  void executeCommand();

public:
  explicit GraphPlanner(rclcpp::Node::SharedPtr& node_handle);

  virtual bool initialize();
  virtual bool execute();
  virtual ~GraphPlanner() = default;
};

#endif  // GRAPH_PLANNER_H
