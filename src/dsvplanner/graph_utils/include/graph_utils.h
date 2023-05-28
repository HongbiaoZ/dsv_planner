/**************************************************************************
graph_utils.h
graph utility functions

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2019
**************************************************************************/
#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <geometry_msgs/msg/point.hpp>
#include "graph_utils/msg/edge.hpp"
#include "graph_utils/msg/topological_graph.hpp"
#include "graph_utils/msg/vertex.hpp"

namespace graph_utils_ns
{
void ShortestPathBtwVertex(std::vector<int>& path, const graph_utils::msg::TopologicalGraph& graph, int start_index,
                           int goal_index);
float PathLength(const std::vector<int>& path, const graph_utils::msg::TopologicalGraph& graph);
int GetClosestVertexIdxToPoint(const graph_utils::msg::TopologicalGraph& graph, const geometry_msgs::msg::Point& pnt);
int GetFirstVertexBeyondThreshold(const geometry_msgs::msg::Point& start_location, const std::vector<int>& path,
                                  const graph_utils::msg::TopologicalGraph& graph, const float distance_threshold);
bool PathCircleDetect(std::vector<int>& path, const graph_utils::msg::TopologicalGraph& graph, int next_vertex_index,
                      geometry_msgs::msg::Point rob_pos);
}

#endif  // GRAPH_UTILS_H
