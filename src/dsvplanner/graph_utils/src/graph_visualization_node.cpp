/**************************************************************************
graph_visualization.cpp
This is the ROS node that calls the graph_visualization

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "graph_visualization.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_handle = rclcpp::Node::make_shared("graph_visualization");

  GraphMarkers graph_markers(node_handle);
  graph_markers.execute();

  return 0;
}
