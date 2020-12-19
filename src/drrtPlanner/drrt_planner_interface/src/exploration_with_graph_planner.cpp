/*
exploration_with_graph_planner.cpp
the interface for drrt planner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
 */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "drrt_planner/drrt_planner_srv.h"
#include "drrt_planner/clean_frontier_srv.h"
#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"

geometry_msgs::Point wayPoint;
geometry_msgs::Point wayPoint_pre;
// geometry_msgs::PointStamped effective_time;
graph_planner::GraphPlannerCommand graph_planner_command;
std_msgs::Float32 effective_time;
std_msgs::Float32 total_time;

bool simulation = false;    // control whether use graph planner to follow path
bool begin_signal = false;  // trigger the planner
bool gp_in_progress = false;
bool wp_state = false;
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
double dtime = 0.0;
double init_x = 2;
double init_y = 0;
double init_z = 2;
std::string map_frame = "map";

ros::Time plan_start;
ros::Time plan_over;

void gp_status_callback(const graph_planner::GraphPlannerStatus::ConstPtr& msg)
{
  if (msg->status == graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS)
    gp_in_progress = true;
  else
  {
    gp_in_progress = false;
  }
}

void waypoint_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  wayPoint = msg->point;
  wp_state = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom_x = msg->pose.pose.position.x;
  current_odom_y = msg->pose.pose.position.y;
  current_odom_z = msg->pose.pose.position.z;
}

void begin_signal_callback(const std_msgs::Bool::ConstPtr& msg)
{
  begin_signal = msg->data;
}

bool wayPointChange(geometry_msgs::Point wp1, geometry_msgs::Point wp2)
{
  double dist =
      sqrt((wp1.x - wp2.x) * (wp1.x - wp2.x) + (wp1.y - wp2.y) * (wp1.y - wp2.y) + (wp1.z - wp2.z) * (wp1.z - wp2.z));
  if (dist < 0.5)
    return false;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  ros::Publisher gp_command_pub = nh.advertise<graph_planner::GraphPlannerCommand>("/graph_planner_command", 1);
  //  ros::Publisher effective_plan_time_pub = nh.advertise<geometry_msgs::PointStamped>("/effective_plan_time", 1);
  ros::Publisher effective_plan_time_pub = nh.advertise<std_msgs::Float32>("/runtime", 1);
  ros::Publisher total_plan_time_pub = nh.advertise<std_msgs::Float32>("/totaltime", 1);
  ros::Subscriber gp_status_sub =
      nh.subscribe<graph_planner::GraphPlannerStatus>("/graph_planner_status", 1, gp_status_callback);
  ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 1, waypoint_callback);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 1, odom_callback);
  ros::Subscriber begin_signal_sub = nh.subscribe<std_msgs::Bool>("/start_exploring", 1, begin_signal_callback);

  ROS_INFO("Start exploration");

  nhPrivate.getParam("simulation", simulation);
  nhPrivate.getParam("/interface/dtime", dtime);
  nhPrivate.getParam("/interface/initX", init_x);
  nhPrivate.getParam("/interface/initY", init_y);
  nhPrivate.getParam("/interface/initZ", init_z);
  nhPrivate.getParam("/interface/tfFrame", map_frame);
  nhPrivate.getParam("/interface/autoExp", begin_signal);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  while (!begin_signal)
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ROS_INFO("Waiting for Odometry");
  }

  ROS_INFO("Starting the planner: Performing initialization motion");
  geometry_msgs::PointStamped wp;
  wp.header.frame_id = map_frame;
  wp.header.stamp = ros::Time::now();
  wp.point.x = init_x + current_odom_x;
  wp.point.y = init_y + current_odom_y;
  wp.point.z = init_z + current_odom_z;

  ros::Duration(0.5).sleep();  // wait for sometime to make sure waypoint can be published properly

  waypoint_pub.publish(wp);
  bool wp_ongoing = true;
  while (wp_ongoing)
  {  // Keep publishing initial waypoint until the robot reaches that point
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    waypoint_pub.publish(wp);
    double dist = sqrt((wp.point.x - current_odom_x) * (wp.point.x - current_odom_x) +
                       (wp.point.y - current_odom_y) * (wp.point.y - current_odom_y));
    if (dist < 0.5)
      wp_ongoing = false;
  }

  total_time.data = 0;
  plan_start = ros::Time::now();
  // Start planning: The planner is called and the computed goal point sent to the graph planner.
  int iteration = 0;
  while (ros::ok())
  {
    if (true)
    {
      ROS_INFO_THROTTLE(0.5, "Planning iteration %i", iteration);
      drrt_planner::drrt_planner_srv planSrv;
      drrt_planner::clean_frontier_srv cleanSrv;
      planSrv.request.header.stamp = ros::Time::now();
      planSrv.request.header.seq = iteration;
      planSrv.request.header.frame_id = map_frame;
      if (ros::service::call("drrtPlannerSrv", planSrv))
      {
        if (planSrv.response.goal.size() == 0)
        {  // usually the size should be 1 if planning successfully
          ros::Duration(1.0).sleep();
          continue;
        }

        plan_over = ros::Time::now();
        //      effective_time.header.stamp = ros::Time::now();
        //      effective_time.point.x = (plan_over - plan_start).toSec();
        effective_time.data = (plan_over - plan_start).toSec();
        effective_plan_time_pub.publish(effective_time);
        total_time.data += effective_time.data;
        total_plan_time_pub.publish(total_time);

        if (!simulation)
        {  // when not in simulation mode, the robot will go to the goal point according to graph planner
          for (int i = 0; i < planSrv.response.goal.size(); i++)
          {
            graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = planSrv.response.goal[i];
            gp_command_pub.publish(graph_planner_command);
            ros::Duration(dtime).sleep();  // give sometime to graph planner for searching path to goal point
            ros::spinOnce();               // update gp_in_progree
            double count = 200;
            while (gp_in_progress)
            {                              // if the waypoint keep the same for 20 (200*0.1)
              ros::Duration(0.1).sleep();  // seconds, then give up the goal
              wayPoint_pre = wayPoint;
              ros::spinOnce();
              bool wpChange = wayPointChange(wayPoint, wayPoint_pre);
              if (wpChange)
              {
                count = 200;
              }
              else
              {
                count--;
              }
              if (count <= 0)
              {  // when the goal point cannot be reached, clean its correspoinding frontier if there is
                cleanSrv.request.header.stamp = ros::Time::now();
                cleanSrv.request.header.frame_id = map_frame;
                ros::service::call("cleanFrontierSrv", cleanSrv);
                ros::Duration(0.1).sleep();
                break;
              }
            }
          }
          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_DISABLE;
          gp_command_pub.publish(graph_planner_command);
        }
        else
        {  // simulation mode is used when testing this planning algorithm with bagfiles where robot will
          // not move to the planned goal. When in simulation mode, robot will keep replanning every two seconds
          for (int i = 0; i < planSrv.response.goal.size(); i++)
          {
            graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = planSrv.response.goal[i];
            gp_command_pub.publish(graph_planner_command);
            ros::Duration(2).sleep();
            break;
          }
        }
        plan_start = ros::Time::now();
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Cannot call drrt planner.");
        ros::Duration(1.0).sleep();
      }
      iteration++;
    }
    else
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }
}
