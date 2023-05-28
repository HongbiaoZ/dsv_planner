/**************************************************************************
dual_state_graph.h
Header of the dual_state_graph class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#ifndef DUAL_STATE_GRAPH_H
#define DUAL_STATE_GRAPH_H

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dsvplanner/grid.h"
#include "graph_planner/msg/graph_planner_status.hpp"
#include "graph_utils/msg/topological_graph.hpp"
#include "octomap_world/octomap_manager.h"

namespace dsvplanner_ns
{
class DualStateGraph
{
public:
  typedef std::shared_ptr<DualStateGraph> Ptr;
  rclcpp::Node::SharedPtr nh_;

  // ROS subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr key_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr graph_planner_path_sub_;
  rclcpp::Subscription<graph_planner::msg::GraphPlannerStatus>::SharedPtr graph_planner_status_sub_;

  // ROS publishers
  rclcpp::Publisher<graph_utils::msg::TopologicalGraph>::SharedPtr local_graph_pub_;
  rclcpp::Publisher<graph_utils::msg::TopologicalGraph>::SharedPtr global_graph_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graph_points_pub_;


  // String constants
  std::string world_frame_id_;
  std::string pub_local_graph_topic_;
  std::string pub_global_graph_topic_;
  std::string pub_global_points_topic_;
  std::string sub_keypose_topic_;
  std::string sub_path_topic_;
  std::string sub_graph_planner_status_topic_;

  // Constants
  bool kCropPathWithTerrain;
  double kConnectVertexDistMax;
  double kDistBadEdge;
  double kDegressiveCoeff;
  double kDirectionCoeff;
  double kExistingPathRatioThreshold;
  double kExistingPathRatioThresholdGlobal;
  double kLongEdgePenaltyMultiplier;
  double kMinVertexDist;
  double kMaxLongEdgeDist;
  double kMaxVertexAngleAlongZ;
  double kMaxVertexDiffAlongZ;
  double kMaxDistToPrunedRoot;
  double kMaxPrunedNodeDist;
  double kSurroundRange;
  double kMinGainRange;
  double kMinDistanceToRobotToCheck;
  Eigen::Vector3d robot_bounding;

  // Variables
  Eigen::Vector3d explore_direction_;
  geometry_msgs::msg::Point robot_pos_;
  geometry_msgs::msg::Pose tempvertex_;
  graph_utils::msg::TopologicalGraph global_graph_;
  graph_utils::msg::TopologicalGraph local_graph_;
  graph_utils::msg::TopologicalGraph pruned_graph_;
  graph_planner::msg::GraphPlannerStatus graph_planner_status_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr graph_point_cloud_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> gainID_;
  volumetric_mapping::OctomapManager* manager_;
  OccupancyGrid* grid_;

  bool planner_status_;  // false means local plan and true means global plan
  int track_localvertex_idx_;
  int track_globalvertex_idx_;
  int prev_track_vertex_idx_;
  int prev_track_keypose_vertex_idx_;

  int localEdgeSize_;
  int globalEdgeSize_;
  int best_vertex_id_;
  double best_gain_;
  double DTWValue_;
  double robot_yaw_;

  // Feedback Functions
  double getGain(geometry_msgs::msg::Point robot_position);
  int getLocalVertexSize();
  int getLocalEdgeSize();
  int getGlobalVertexSize();
  int getGlobalEdgeSize();
  geometry_msgs::msg::Point getBestLocalVertexPosition();
  geometry_msgs::msg::Point getBestGlobalVertexPosition();
  Eigen::Vector3d getExploreDirection();

  // General Functions
  void addEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::msg::TopologicalGraph& graph);
  void addEdge(int start_vertex_idx, int end_vertex_idx, graph_utils::msg::TopologicalGraph& graph);
  void addNewLocalVertex(geometry_msgs::msg::Pose& vertex_msg, graph_utils::msg::TopologicalGraph& graph);
  void addNewLocalVertexWithoutEdge(geometry_msgs::msg::Pose& vertex_msg, graph_utils::msg::TopologicalGraph& graph);
  void addNewLocalVertexWithoutDuplicates(geometry_msgs::msg::Pose& vertex_msg, graph_utils::msg::TopologicalGraph& graph);
  void addNewPrunedVertex(geometry_msgs::msg::Pose& vertex_msg, graph_utils::msg::TopologicalGraph& graph);
  void addGlobalEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, bool trajectory_edge);
  void addGlobalEdge(int start_vertex_idx, int end_vertex_idx);
  void addNewGlobalVertex(geometry_msgs::msg::Pose& vertex_msg);
  void addNewGlobalVertexWithoutDuplicates(geometry_msgs::msg::Pose& vertex_msg);
  void addNewGlobalVertexWithKeypose(geometry_msgs::msg::Pose& vertex_msg);
  void clearLocalGraph();
  void DTW(std::vector<int> path, geometry_msgs::msg::Point robot_position);
  void pruneGraph(geometry_msgs::msg::Point root);
  void pruneGlobalGraph();
  void publishLocalGraph();
  void publishGlobalGraph();
  void setCurrentPlannerStatus(bool status);
  void updateGlobalGraph();
  void updateExploreDirection();

  bool zCollisionCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::msg::TopologicalGraph graph);

  // Callback Functions
  void keyposeCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr graph_path);
  void graphPlannerStatusCallback(const graph_planner::msg::GraphPlannerStatus::SharedPtr status);

public:
  DualStateGraph(rclcpp::Node::SharedPtr& node_handle,
                 volumetric_mapping::OctomapManager* manager, OccupancyGrid* grid);
  bool readParameters();
  bool initialize();
  bool execute();
  ~DualStateGraph();
};
}
#endif  // DUAL_STATE_GRAPH_H
