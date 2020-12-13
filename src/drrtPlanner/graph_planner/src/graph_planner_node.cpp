/**************************************************************************
graph_planner.cpp
This is the ROS node that calls the graph_planner

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "graph_planner.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_planner");
    ros::NodeHandle node_handle("~");

    graph_planner_ns::GraphPlanner graph_planner(node_handle);
    graph_planner.execute();

    return 0;
}
