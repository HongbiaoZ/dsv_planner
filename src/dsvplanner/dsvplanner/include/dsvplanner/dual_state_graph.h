/**************************************************************************
dual_state_graph.h
Header of the dual_state_graph class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#ifndef DUAL_STATE_GRAPH_H
#define DUAL_STATE_GRAPH_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "dsvplanner/grid.h"
#include "graph_planner/GraphPlannerStatus.h"
#include "graph_utils/TopologicalGraph.h"
#include "octomap_world/octomap_manager.h"

namespace dsvplanner_ns {
class DualStateGraph {
public:
  typedef std::shared_ptr<DualStateGraph> Ptr;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS subscribers
  ros::Subscriber key_pose_sub_;
  ros::Subscriber graph_planner_path_sub_;
  ros::Subscriber graph_planner_status_sub_;

  // ROS publishers
  ros::Publisher local_graph_pub_;
  ros::Publisher global_graph_pub_;
  ros::Publisher graph_points_pub_;

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
  Eigen::Vector3d robot_bounding;

  // Variables
  Eigen::Vector3d explore_direction_;
  geometry_msgs::Point robot_pos_;
  geometry_msgs::Pose tempvertex_;
  graph_utils::TopologicalGraph global_graph_;
  graph_utils::TopologicalGraph local_graph_;
  graph_utils::TopologicalGraph pruned_graph_;
  graph_planner::GraphPlannerStatus graph_planner_status_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr graph_point_cloud_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> gainID_;
  volumetric_mapping::OctomapManager *manager_;
  OccupancyGrid *grid_;

  bool planner_status_; // false means local plan and true means global plan
  int track_localvertex_idx_;
  int track_globalvertex_idx_;
  int prev_track_vertex_idx_;

  int localEdgeSize_;
  int globalEdgeSize_;
  int best_vertex_id_;
  double best_gain_;
  double DTWValue_;
  double robot_yaw_;

  // Feedback Functions
  double getGain(geometry_msgs::Point robot_position);
  int getLocalVertexSize();
  int getLocalEdgeSize();
  int getGlobalVertexSize();
  int getGlobalEdgeSize();
  geometry_msgs::Point getBestLocalVertexPosition();
  geometry_msgs::Point getBestGlobalVertexPosition();
  Eigen::Vector3d getExploreDirection();

  // General Functions
  void addEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx,
                           graph_utils::TopologicalGraph &graph);
  void addEdge(int start_vertex_idx, int end_vertex_idx,
               graph_utils::TopologicalGraph &graph);
  void addNewLocalVertex(geometry_msgs::Pose &vertex_msg,
                         graph_utils::TopologicalGraph &graph);
  void addNewLocalVertexWithoutEdge(geometry_msgs::Pose &vertex_msg,
                                    graph_utils::TopologicalGraph &graph);
  void addNewLocalVertexWithoutDuplicates(geometry_msgs::Pose &vertex_msg,
                                          graph_utils::TopologicalGraph &graph);
  void addNewPrunedVertex(geometry_msgs::Pose &vertex_msg,
                          graph_utils::TopologicalGraph &graph);
  void addGlobalEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx);
  void addGlobalEdge(int start_vertex_idx, int end_vertex_idx);
  void addNewGlobalVertex(geometry_msgs::Pose &vertex_msg);
  void addNewGlobalVertexWithoutDuplicates(geometry_msgs::Pose &vertex_msg);
  void addNewGlobalVertexWithKeypose(geometry_msgs::Pose &vertex_msg);
  void clearLocalGraph();
  void DTW(std::vector<int> path, geometry_msgs::Point robot_position);
  void pruneGraph(geometry_msgs::Point root);
  void pruneGlobalGraph();
  void publishLocalGraph();
  void publishGlobalGraph();
  void setCurrentPlannerStatus(bool status);
  void updateGlobalGraph();
  void updateExploreDirection();

  bool zCollisionCheck(int start_vertex_idx, int end_vertex_idx,
                       graph_utils::TopologicalGraph graph);

  // Callback Functions
  void keyposeCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void pathCallback(const nav_msgs::Path::ConstPtr &graph_path);
  void graphPlannerStatusCallback(
      const graph_planner::GraphPlannerStatusConstPtr &status);

public:
  DualStateGraph(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 volumetric_mapping::OctomapManager *manager,
                 OccupancyGrid *grid);
  bool readParameters();
  bool initialize();
  bool execute();
  ~DualStateGraph();
};
}
#endif // DUAL_STATE_GRAPH_H
