/**************************************************************************
graph_planner.cpp
This is the ROS node that calls the graph_planner

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "graph_planner.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_handle = rclcpp::Node::make_shared("graph_planner");

  graph_planner_ns::GraphPlanner graph_planner(node_handle);
  graph_planner.execute();

  return 0;
}
