/**************************************************************************
graph_visualization.cpp

This graph listens for TopologicalGraph messages
Converts each to a marker message (for rviz), then publishes it

Modified and maintained by:
HongbiaoZhu (hongbiaz@andrew.cmu.edu)
**************************************************************************/

#include "graph_visualization.h"

bool GraphMarkers::readParameters() {
  if (!nh_.getParam("sub_local_graph_topic_", sub_local_graph_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_local_graph_topic_");
    return false;
  }
  if (!nh_.getParam("pub_local_graph_marker_topic_", pub_local_graph_marker_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_local_graph_marker_topic_");
    return false;
  }
  if (!nh_.getParam("sub_global_graph_topic_", sub_global_graph_topic_)) {
    ROS_ERROR("Cannot read parameter: sub_global_graph_topic_");
    return false;
  }
  if (!nh_.getParam("pub_global_graph_marker_topic_", pub_global_graph_marker_topic_)) {
    ROS_ERROR("Cannot read parameter: pub_global_graph_marker_topic_");
    return false;
  }
  return true;
}


void GraphMarkers::initializeMarkers() {

  local_graph_vertex_marker_.ns = "graph vertex";
  local_graph_vertex_marker_.action = visualization_msgs::Marker::ADD;
  local_graph_vertex_marker_.pose.orientation.w = 1.0;
  local_graph_vertex_marker_.id = 1;
  local_graph_vertex_marker_.type = visualization_msgs::Marker::POINTS;
  local_graph_vertex_marker_.scale.x = 0.2;
  local_graph_vertex_marker_.color.a = 1.0;
  local_graph_vertex_marker_.color.g = 0.0;
  local_graph_vertex_marker_.color.r = 1.0;

  local_graph_edge_marker_.ns = "graph edge";
  local_graph_edge_marker_.action = visualization_msgs::Marker::ADD;
  local_graph_edge_marker_.pose.orientation.w = 1.0;
  local_graph_edge_marker_.id = 1;
  local_graph_edge_marker_.type = visualization_msgs::Marker::LINE_LIST;
  local_graph_edge_marker_.scale.x = 0.03;
  local_graph_edge_marker_.color.a = 0.9;
  local_graph_edge_marker_.color.g = 1.0;
  local_graph_edge_marker_.color.r = 1.0;

  global_graph_vertex_marker_.ns = "graph vertex";
  global_graph_vertex_marker_.action = visualization_msgs::Marker::ADD;
  global_graph_vertex_marker_.pose.orientation.w = 1.0;
  global_graph_vertex_marker_.id = 1;
  global_graph_vertex_marker_.type = visualization_msgs::Marker::POINTS;
  global_graph_vertex_marker_.scale.x = 0.2;
  global_graph_vertex_marker_.color.a = 0.5;
  global_graph_vertex_marker_.color.g = 1.0;
  global_graph_vertex_marker_.color.r = 0.5;

  global_graph_edge_marker_.ns = "graph edge";
  global_graph_edge_marker_.action = visualization_msgs::Marker::ADD;
  global_graph_edge_marker_.pose.orientation.w = 1.0;
  global_graph_edge_marker_.id = 1;
  global_graph_edge_marker_.type = visualization_msgs::Marker::LINE_LIST;
  global_graph_edge_marker_.scale.x = 0.03;
  global_graph_edge_marker_.color.a = 0.9;
  global_graph_edge_marker_.color.g = 1.0;
  global_graph_edge_marker_.color.r = 0.0;

}

void GraphMarkers::topologicalLocalGraphCallback(const graph_utils::TopologicalGraph::ConstPtr& graph_msg) {

	// Store this message -- periodically process it in PublishMarker()
  topological_local_graph_ = *graph_msg;
  new_local_graph_received_ = true;
}

void GraphMarkers::topologicalGlobalGraphCallback(const graph_utils::TopologicalGraph::ConstPtr &graph_msg)
{
  topological_global_graph_ = *graph_msg;
  new_global_graph_received_ = true;
}

void GraphMarkers::generateMarkers() {

	// Reset the headers
  local_graph_vertex_marker_.header = topological_local_graph_.header;
  local_graph_edge_marker_.header = topological_local_graph_.header;

  // Clear the markers
  local_graph_vertex_marker_.points.clear();
  local_graph_edge_marker_.points.clear();

  // Iterate through all vertices of the graph
  for(const auto & vertex : topological_local_graph_.vertices) {

  	// Add marker for this vertex
    local_graph_vertex_marker_.points.push_back(vertex.location);

    // Iterate through adjacent edges of this vertex
    for(const auto & edge : vertex.edges) {

    	// Add marker for this edge
      int to_idx = edge.vertex_id_end;
      local_graph_edge_marker_.points.push_back(vertex.location);
      local_graph_edge_marker_.points.push_back(topological_local_graph_.vertices[to_idx].location);
    }
  }
}

void GraphMarkers::generateGlobalMarkers()
{
  // Reset the headers
  global_graph_vertex_marker_.header = topological_local_graph_.header;
  global_graph_edge_marker_.header = topological_local_graph_.header;

  // Clear the markers
  global_graph_vertex_marker_.points.clear();
  global_graph_edge_marker_.points.clear();

  // Iterate through all vertices of the graph
  for(const auto & vertex : topological_global_graph_.vertices) {

  	// Add marker for this vertex
    global_graph_vertex_marker_.points.push_back(vertex.location);

    // Iterate through adjacent edges of this vertex
    for(const auto & edge : vertex.edges) {

    	// Add marker for this edge
      int to_idx = edge.vertex_id_end;
      global_graph_edge_marker_.points.push_back(vertex.location);
      global_graph_edge_marker_.points.push_back(topological_global_graph_.vertices[to_idx].location);
    }
  }
}

void GraphMarkers::publishMarkers() {

  // Convert TopologicalGraph message to a marker
	// and publish it
  generateMarkers();  

  // Publish the markers
  local_graph_marker_pub_.publish(local_graph_vertex_marker_);
  local_graph_marker_pub_.publish(local_graph_edge_marker_);

  // Don't redraw until new graph received
  new_local_graph_received_ = false;
}

void GraphMarkers::publishGlobalMarkers()
{
  generateGlobalMarkers();

  // Publish the markers
  global_graph_marker_pub_.publish(global_graph_vertex_marker_);
  global_graph_marker_pub_.publish(global_graph_edge_marker_);

  // Don't redraw until new graph received
  new_global_graph_received_ = false;
}

bool GraphMarkers::execute() {

  // At the specified execute frequency (see launch file),
  // generate and publish the marker
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {

    ros::spinOnce();
    if(new_local_graph_received_) {
      publishMarkers();
    }

    if(new_global_graph_received_)
      publishGlobalMarkers();

    status = ros::ok();
    rate.sleep();

  }
  return true;
}

bool GraphMarkers::initialize() {

	// Read in parameters
  if (!readParameters()) 
  	return false;

  // Initialize subscribers
  local_graph_sub_ = nh_.subscribe(sub_local_graph_topic_, 1, &GraphMarkers::topologicalLocalGraphCallback, this);
  global_graph_sub_ = nh_.subscribe(sub_global_graph_topic_, 1, &GraphMarkers::topologicalGlobalGraphCallback, this);



  // Initialize publishers
  local_graph_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(pub_local_graph_marker_topic_, 10);
  global_graph_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(pub_global_graph_marker_topic_, 10);

  // Initialize markers
  initializeMarkers();

  ROS_INFO("Successfully launched GraphVisualization node");

  return true;
}

GraphMarkers::GraphMarkers(const ros::NodeHandle& nh){
  nh_ = nh;
  initialize();
}

