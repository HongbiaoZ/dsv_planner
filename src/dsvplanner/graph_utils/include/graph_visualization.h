/**************************************************************************
graph_visualization.h

Created by Chao Cao (ccao1@andrew.cmu.edu)
6/3/19

Modified and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "graph_utils/TopologicalGraph.h"

class GraphMarkers {
 private:

  // ROS handler
  ros::NodeHandle nh_;

  // ROS subscribers
  ros::Subscriber local_graph_sub_;
  ros::Subscriber global_graph_sub_;

  // ROS publishers
  ros::Publisher local_graph_marker_pub_;
  ros::Publisher global_graph_marker_pub_;

  // String constants
  std::string sub_local_graph_topic_;
  std::string pub_local_graph_marker_topic_;
  std::string sub_global_graph_topic_;
  std::string pub_global_graph_marker_topic_;

  // Markers that will be published
  visualization_msgs::Marker local_graph_vertex_marker_;
  visualization_msgs::Marker local_graph_edge_marker_;

  visualization_msgs::Marker global_graph_vertex_marker_;
  visualization_msgs::Marker global_graph_edge_marker_;

  graph_utils::TopologicalGraph topological_local_graph_;
  graph_utils::TopologicalGraph topological_global_graph_;
  bool new_local_graph_received_ = false;
  bool new_global_graph_received_ = false;

  bool readParameters();
  void initializeMarkers();
  void topologicalLocalGraphCallback(const graph_utils::TopologicalGraph::ConstPtr& graph_msg);
  void topologicalGlobalGraphCallback(const graph_utils::TopologicalGraph::ConstPtr& graph_msg);
  void generateMarkers();
  void generateGlobalMarkers();
  void publishMarkers();
  void publishGlobalMarkers();  

 public:
  explicit GraphMarkers(const ros::NodeHandle& nh);

  virtual bool initialize();
  virtual bool execute();
  virtual ~GraphMarkers() = default;
};
