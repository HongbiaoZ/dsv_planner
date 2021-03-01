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

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"
#include "graph_utils/Edge.h"
#include "graph_utils/TopologicalGraph.h"
#include "graph_utils/Vertex.h"

namespace graph_planner_ns {
typedef pcl::PointXYZ PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLCloudType;

class GraphPlanner;
typedef struct {
  bool in_polygon;
  float cost_estimate;
} VertexScore;
} // namespace dfs_behavior_planner_ns

class graph_planner_ns::GraphPlanner {
private:
  // ROS Nodehandler
  ros::NodeHandle nh_;

  // ROS subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber graph_sub_;
  ros::Subscriber graph_planner_command_sub_;

  // ROS publishers
  ros::Publisher waypoint_pub_;
  ros::Publisher graph_planner_status_pub_;
  ros::Publisher graph_planner_path_pub_;

  // String constants
  std::string world_frame_id_;
  std::string sub_odometry_topic_;
  std::string sub_graph_topic_;
  std::string graph_planner_command_topic_;
  std::string graph_planner_status_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_path_topic_;

  // Constants
  float kLookAheadDist;
  double kWaypointProjectionDistance;
  double kObstacleHeightThres;
  double kOverheadObstacleHeightThres;
  double kCollisionCheckDistace;

  // "State" variables
  enum GraphPlannerState { ALL_OTHER_STATES = 0, NEARBY_OPENING = 1 };
  GraphPlannerState state_;

  // Variables
  graph_planner::GraphPlannerCommand graph_planner_command_; // received command
  graph_utils::TopologicalGraph
      planned_graph_; // the received graph that can be palnned path in
  geometry_msgs::PointStamped waypoint_; // goal waypoint being published by
                                         // this node (if in mode IN_CONTROL)
  geometry_msgs::Point robot_pos_;       // current robot position
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_point_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_point_crop_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<geometry_msgs::Point> planned_path_; // only used for debugging --
                                                   // the waypoint_ is what is
                                                   // actually
                                                   // output
  double robot_yaw_;                               // current robot yaw
  bool in_progress_ = false; // current graph planner status
  bool wrong_id_ = false;
  int backTraceCount_ = 0;
  int previous_shortest_path_size_ = 100000; // the size of the previous planned
                                             // path. used to avoid moving back
                                             // and forth in some cases
  int wrong_id_shortest_path_size_ = 0;
  int previous_shortest_path_size_when_pathrewind = 100000;
  int previous_vertex_id_; // the id of the previous planned goal vertex

  // Callbacks
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry_msg);
  void graphCallback(const graph_utils::TopologicalGraph::ConstPtr &graph_msg);
  void commandCallback(const graph_planner::GraphPlannerCommand::ConstPtr &msg);
  void terrainCallback(const sensor_msgs::PointCloud2::ConstPtr &terrain_msg);
  // Other functions
  void publishPath();
  void alterAndPublishWaypoint();
  void publishInProgress(bool in_progress);
  bool readParameters();
  bool goToVertex(int current_vertex_idx, int goal_vertex_idx);
  bool goToPoint(geometry_msgs::Point point);
  bool collisionCheckByTerrain(geometry_msgs::Point robot_position,
                               int end_vertex_idx);
  geometry_msgs::Point projectWayPoint(geometry_msgs::Point next_vertex_pos,
                                       geometry_msgs::Point robot_pos);

  // Execute function variants
  void executeGoToOrigin();   // for returning home
  void executeGoToLocation(); // for going to goal point
  void executeCommand();

public:
  explicit GraphPlanner(const ros::NodeHandle &nh);

  virtual bool initialize();
  virtual bool execute();
  virtual ~GraphPlanner() = default;
};

#endif // GRAPH_PLANNER_H
