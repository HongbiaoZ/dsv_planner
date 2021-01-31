#include "graph_utils.h"

#include <queue>
#include <vector>

#include <misc_utils/misc_utils.h>

#include <math.h>
#include <bits/stdc++.h>
#define INF 0x3f3f3f3f  // integer infinity
#define PI 3.14159265358979323846
using namespace std;

namespace graph_utils_ns
{
// Function for getting the shortest path on a graph between two vertexes
// Input: graph, index of the start vertex and index of the goal vertex
// Output: a sequence of vertex ids as the path
void ShortestPathBtwVertex(vector<int>& path, const graph_utils::TopologicalGraph& graph, int start_index,
                           int goal_index)
{
  if (start_index == goal_index)
  {
    path.clear();
    path.push_back(start_index);
    return;
  }
  // Vertices are represented by their index in the graph.vertices list
  typedef pair<float, int> iPair;

  // Priority queue of vertices
  priority_queue<iPair, vector<iPair>, greater<iPair> > pq;

  // Vector of distances
  vector<float> dist(graph.vertices.size(), INFINITY);

  // Vector of backpointers
  vector<int> backpointers(graph.vertices.size(), INF);

  // Add the start vertex
  pq.push(make_pair(0, start_index));
  dist[start_index] = 0;

  // Loop until priority queue is empty
  while (!pq.empty())
  {
    // Pop the minimum distance vertex
    int u = pq.top().second;
    pq.pop();

    // Get all adjacent vertices
    for (auto it = graph.vertices[u].edges.begin(); it != graph.vertices[u].edges.end(); ++it)
    {
      // Get vertex label and weight of current adjacent edge of u
      int v = it->vertex_id_end;
      float weight = it->traversal_costs;

      // If there is a shorter path to v through u
      if (dist[v] > dist[u] + weight)
      {
        // Updating distance of v
        dist[v] = dist[u] + weight;
        pq.push(make_pair(dist[v], v));
        backpointers[v] = u;
      }
    }

    // Early termination
    if (u == goal_index)
    {
      break;
    }
  }

  // Backtrack to find path
  vector<int> reverse_path;
  int current = goal_index;
  if (backpointers[current] == INF)
  {
    // no path found
    // std::cout << "WARNING: no path found " << start_index << "<->" << goal_index << std::endl;
    path.clear();
  }
  else
  {
    // path found

    while (current != INF)
    {
      reverse_path.push_back(current);
      current = backpointers[current];
    }

    // Reverse the path (constructing it this way since vector is more efficient at push_back than insert[0])
    path.clear();
    for (int i = reverse_path.size() - 1; i >= 0; --i)
    {
      path.push_back(reverse_path[i]);
    }
  }
}

/// Compute path length, where path represented by sequence of vertex indices
float PathLength(const vector<int>& path, const graph_utils::TopologicalGraph& graph)
{
  float cost = 0;
  if (path.size() == 0)
  {
    std::cout << "WARNING: PathLength queried for empty path" << std::endl;
    return 0;
  }

  for (int i = 0; i < -1 + path.size(); i++)
  {
    int index1 = path[i];
    int index2 = path[i + 1];

    bool found = false;

    // search for the edge
    for (auto it = graph.vertices[index1].edges.begin(); it != graph.vertices[index1].edges.end(); ++it)
    {
      if (it->vertex_id_end == index2)
      {
        cost += it->traversal_costs;
        found = true;
        break;
      }
    }
    if (!found)
      std::cout << "WARNING: edge " << index1 << "<->" << index2 << " not found" << std::endl;
  }
  return cost;
}

/// Find the vertex idx in graph that is closest (Euclidean distance) to pnt
int GetClosestVertexIdxToPoint(const graph_utils::TopologicalGraph& graph, const geometry_msgs::Point& pnt)
{
  double best_idx = -1;
  double best_distance = INFINITY;
  for (int v_idx = 0; v_idx < graph.vertices.size(); ++v_idx)
  {
    double distance = misc_utils_ns::PointXYZDist(pnt, graph.vertices[v_idx].location);
    if (distance < best_distance && fabs(pnt.z - graph.vertices[v_idx].location.z) < 1.0)
    {
      best_idx = v_idx;
      best_distance = distance;
    }
  }
  // std::cout << "The closest vertex is "<< best_idx<< std::endl;
  return best_idx;
}

/// Returns the vertex_index of the first vertex along the path that is beyond threshold distance from the 1st vertex on
/// path
/// if none exist, it returns the last vertex
/// To be considered, a vertex must have BOTH accumulated and Euclidean distance away
/// Euclidean distance only relevant if path wraps back on itself (shouldn't happen if it's a "shortest path")
/// Assumes path is not empty
int GetFirstVertexBeyondThreshold(const geometry_msgs::Point& start_location, const std::vector<int>& path,
                                  const graph_utils::TopologicalGraph& graph, const float distance_threshold)
{
  if (path.size() == 1)
  {
    // trivial case
    return path[0];
  }

  // Start with distance to first vertex
  auto first_vertex_location = graph.vertices[path[0]].location;
  double distance_along_path = misc_utils_ns::PointXYZDist(start_location, first_vertex_location);

  // Move along path, accumulating the distance
  // Pick first
  for (int i = 1; i < path.size(); i++)
  {
    // Extract consecutive pair of graph locations on path
    int v_idx = path[i];
    auto vertex_location_prev = graph.vertices[path[i - 1]].location;
    auto vertex_location = graph.vertices[path[i]].location;

    // Accumulate distance
    distance_along_path += misc_utils_ns::PointXYZDist(vertex_location, vertex_location_prev);

    // Get Euclidean distance from start
    double distance_euclidean = misc_utils_ns::PointXYZDist(start_location, vertex_location);

    // If distance threshold exceeded using BOTH measures
    if ((distance_along_path >= distance_threshold) && (distance_euclidean >= distance_threshold))
    {
      return v_idx;
    }
  }

  // if none are above the threshold, return index of last point on path
  return path.back();
}

bool PathCircleDetect(std::vector<int>& path, const graph_utils::TopologicalGraph& graph, int next_vertex_index)
{
  double accumulated_angle_difference = 0;
  geometry_msgs::Point pointA, pointB, pointC;
  std::vector<int>::iterator it = std::find(path.begin(), path.end(), next_vertex_index);
  if (it != path.end())
  {
    int index = it - path.begin();
    if (index >= 3)
    {
      for (int i = 2; i < index + 1; i++)
      {
        pointA = graph.vertices[path[i - 2]].location;
        pointB = graph.vertices[path[i - 1]].location;
        pointC = graph.vertices[path[i]].location;
        double angle1 = atan2(pointB.y - pointA.y, pointB.x - pointA.x);
        double angle2 = atan2(pointC.y - pointB.y, pointC.x - pointB.x);
        double angle_difference = angle2 - angle1;
        if (angle_difference > PI)
        {
          angle_difference = 2 * PI - angle_difference;
        }
        else if (angle_difference < -PI)
        {
          angle_difference = 2 * PI + angle_difference;
        }
        accumulated_angle_difference += angle_difference;
      }
      if (std::fabs(accumulated_angle_difference) > 2.0 / 3 * PI)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }
  else
  {
    return false;
  }
}
}
