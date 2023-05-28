/**************************************************************************
graph_visualization.h

Created by Chao Cao (ccao1@andrew.cmu.edu)
6/3/19

Modified and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "graph_utils/msg/topological_graph.hpp"

class GraphMarkers
{
private:
  // ROS handler
  rclcpp::Node::SharedPtr nh_;

  // ROS subscribers
  rclcpp::Subscription<graph_utils::msg::TopologicalGraph>::SharedPtr local_graph_sub_;
  rclcpp::Subscription<graph_utils::msg::TopologicalGraph>::SharedPtr global_graph_sub_;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_graph_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_graph_marker_pub_;

  // String constants
  std::string sub_local_graph_topic_;
  std::string pub_local_graph_marker_topic_;
  std::string sub_global_graph_topic_;
  std::string pub_global_graph_marker_topic_;

  // Markers that will be published
  visualization_msgs::msg::Marker local_graph_vertex_marker_;
  visualization_msgs::msg::Marker local_graph_edge_marker_;

  visualization_msgs::msg::Marker global_graph_vertex_marker_;
  visualization_msgs::msg::Marker global_graph_edge_marker_;

  graph_utils::msg::TopologicalGraph topological_local_graph_;
  graph_utils::msg::TopologicalGraph topological_global_graph_;
  bool new_local_graph_received_ = false;
  bool new_global_graph_received_ = false;

  bool readParameters();
  void initializeMarkers();
  void topologicalLocalGraphCallback(const graph_utils::msg::TopologicalGraph::SharedPtr graph_msg);
  void topologicalGlobalGraphCallback(const graph_utils::msg::TopologicalGraph::SharedPtr graph_msg);
  void generateMarkers();
  void generateGlobalMarkers();
  void publishMarkers();
  void publishGlobalMarkers();

public:
  explicit GraphMarkers(rclcpp::Node::SharedPtr& node_handle);

  virtual bool initialize();
  virtual bool execute();
  virtual ~GraphMarkers() = default;
};
