/**************************************************************************
graph_visualization.cpp
This is the ROS node that calls the graph_visualization

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "graph_visualization.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_visualization");
    ros::NodeHandle node_handle("~");

    GraphMarkers graph_markers(node_handle);
    graph_markers.execute();

    return 0;
}
