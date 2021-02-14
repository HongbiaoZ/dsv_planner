/**************************************************************************
graph_utils.h
graph utility functions

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2019
**************************************************************************/
#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "graph_utils/Edge.h"
#include "graph_utils/TopologicalGraph.h"
#include "graph_utils/Vertex.h"

namespace graph_utils_ns {
void ShortestPathBtwVertex(std::vector<int> &path,
                           const graph_utils::TopologicalGraph &graph,
                           int start_index, int goal_index);
float PathLength(const std::vector<int> &path,
                 const graph_utils::TopologicalGraph &graph);
int GetClosestVertexIdxToPoint(const graph_utils::TopologicalGraph &graph,
                               const geometry_msgs::Point &pnt);
int GetFirstVertexBeyondThreshold(const geometry_msgs::Point &start_location,
                                  const std::vector<int> &path,
                                  const graph_utils::TopologicalGraph &graph,
                                  const float distance_threshold);
bool PathCircleDetect(std::vector<int> &path,
                      const graph_utils::TopologicalGraph &graph,
                      int next_vertex_index, geometry_msgs::Point rob_pos);
}

#endif // GRAPH_UTILS_H
