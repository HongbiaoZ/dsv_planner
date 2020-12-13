/*
drrtp_node.cpp
node to launch drrtplanner

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <drrt_planner/drrtp.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nbvPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  drrt_planner_ns::drrtPlanner planner(nh, nh_private);

  ros::spin();
  return 0;
}
