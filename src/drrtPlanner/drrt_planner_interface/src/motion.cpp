/*
data_collect_gbp.cpp
planner data processing for gbplanner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <iostream>
#include <string>

using namespace std;
ros::Time time_start;
ros::Time current_time;

nav_msgs::Odometry previous_odom;
nav_msgs::Odometry current_odom;
bool first_iteration = true;

double current_twist;
double previous_twist;
double current_angular_twist;
double previous_angular_twist;

double max_speed = 0;
double min_speed = 0;
double max_acc = 0;
double min_acc = 0;
double max_angular_speed = 0;
double min_angular_speed = 0;
double max_angular_acc = 0;
double min_angular_acc = 0;

std::string motion_data_name;
ofstream motion_data_file;
std::ostream& out1 = motion_data_file;


void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(first_iteration == true){
    current_odom = *msg;
    first_iteration = false;
  }
  if(first_iteration == false)
  {
    previous_odom = current_odom;
    current_odom = *msg;
    current_time = msg->header.stamp;
    time_start = previous_odom.header.stamp;
    current_twist = msg->twist.twist.linear.x;
    previous_twist = previous_odom.twist.twist.linear.x;
    current_angular_twist = msg->twist.twist.angular.z;
    previous_angular_twist = previous_odom.twist.twist.angular.z;
    double timeInterval = (current_time - time_start).toSec();
    double velChange = current_twist - previous_twist;
    double angular_velchange = current_angular_twist - previous_angular_twist;
    double acc = velChange/timeInterval;
    double angular_acc = angular_velchange/timeInterval;
    if(current_twist>max_speed)
      max_speed = current_twist;
    if(current_twist<min_speed)
      min_speed = current_twist;
    if(acc>max_acc)
      max_acc = acc;
    if(acc<min_acc)
      min_acc = acc;
    if(current_angular_twist>max_angular_speed)
      max_angular_speed = current_angular_twist;
    if(current_angular_twist<min_angular_speed)
      min_angular_speed = current_angular_twist;
    if(angular_acc>max_angular_acc)
      max_angular_acc = angular_acc;
    if(angular_acc<min_angular_acc)
      min_angular_acc = angular_acc;
    out1<<current_twist<<"  "<<acc<<"  "<<current_angular_twist<<"  "<<angular_acc<<endl;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_with_lp");
  ros::NodeHandle nh;
  ros::Subscriber odometry_sub = nh.subscribe < nav_msgs::Odometry > ("/integrated_to_map", 1, OdometryCallback);

  ROS_INFO("Started Timing");

  motion_data_name = ros::package::getPath("interface_nbvp_rotors") + "/data/motion.txt";
  motion_data_file.open(motion_data_name);
  motion_data_file.app;
  out1<<"linear_speed linear_acc angular_speed angular_acc"<<endl;


  // Start planning: The planner is called and the computed path sent to the controller.
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    std::cout<<"min_speed= "<<min_speed<<" max_speed= "<<max_speed<<" min_acc= "<<min_acc<<" max_acc= "<<max_acc<<std::endl;
    std::cout<<"min_angular_speed= "<<min_angular_speed<<" max_angular_speed= "<<max_angular_speed<<" min_angular_acc= "<<min_angular_acc<<" max_angular_acc= "<<max_angular_acc<<std::endl;
    loop_rate.sleep();
  }
  return -1;
}
