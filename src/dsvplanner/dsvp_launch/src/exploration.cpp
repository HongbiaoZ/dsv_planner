/*
exploration_with_graph_planner.cpp
the interface for drrt planner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
 */

#include <chrono>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>

#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"

using namespace std::chrono;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

geometry_msgs::Point wayPoint;
geometry_msgs::Point wayPoint_pre;
geometry_msgs::Point goal_point;
geometry_msgs::Point home_point;
graph_planner::GraphPlannerCommand graph_planner_command;
std_msgs::Float32 effective_time;
std_msgs::Float32 total_time;

bool simulation = false;    // control whether use graph planner to follow path
bool begin_signal = false;  // trigger the planner
bool gp_in_progress = false;
bool wp_state = false;
bool return_home = false;
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
double previous_odom_x = 0;
double previous_odom_y = 0;
double previous_odom_z = 0;
double dtime = 0.0;
double init_x = 2;
double init_y = 0;
double init_z = 2;
double init_time = 2;
double return_home_threshold = 1.5;
double robot_moving_threshold = 6;
std::string map_frame = "map";
std::string waypoint_topic = "/way_point";
std::string cmd_vel_topic = "/cmd_vel";
std::string gp_command_topic = "/graph_planner_command";
std::string effective_plan_time_topic = "/runtime";
std::string total_plan_time_topic = "/totaltime";
std::string gp_status_topic = "/graph_planner_status";
std::string odom_topic = "/state_estimation";
std::string begin_signal_topic = "/start_exploring";
std::string stop_signal_topic = "/stop_exploring";

tf::StampedTransform transformToMap;

steady_clock::time_point plan_start;
steady_clock::time_point plan_over;
steady_clock::duration time_span;

ros::Publisher waypoint_pub;
ros::Publisher gp_command_pub;
ros::Publisher effective_plan_time_pub;
ros::Publisher total_plan_time_pub;
ros::Subscriber gp_status_sub;
ros::Subscriber waypoint_sub;
ros::Subscriber odom_sub;
ros::Subscriber begin_signal_sub;
ros::Publisher stop_signal_pub;

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

  transformToMap.setOrigin(
      tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

void begin_signal_callback(const std_msgs::Bool::ConstPtr& msg)
{
  begin_signal = msg->data;
}

bool robotPositionChange()
{
  double dist = sqrt((current_odom_x - previous_odom_x) * (current_odom_x - previous_odom_x) +
                     (current_odom_y - previous_odom_y) * (current_odom_y - previous_odom_y) +
                     (current_odom_z - previous_odom_z) * (current_odom_z - previous_odom_z));
  if (dist < robot_moving_threshold)
    return false;
  previous_odom_x = current_odom_x;
  previous_odom_y = current_odom_y;
  previous_odom_z = current_odom_z;
  return true;
}

void initilization()
{
  tf::Vector3 vec_init(init_x, init_y, init_z);
  tf::Vector3 vec_goal;
  vec_goal = transformToMap * vec_init;
  geometry_msgs::PointStamped wp;
  wp.header.frame_id = map_frame;
  wp.header.stamp = ros::Time::now();
  wp.point.x = vec_goal.x();
  wp.point.y = vec_goal.y();
  wp.point.z = vec_goal.z();
  home_point.x = current_odom_x;
  home_point.y = current_odom_y;
  home_point.z = current_odom_z;

  ros::Duration(0.5).sleep();  // wait for sometime to make sure waypoint can be
                               // published properly

  waypoint_pub.publish(wp);
  bool wp_ongoing = true;
  int init_time_count = 0;
  while (wp_ongoing)
  {  // Keep publishing initial waypoint until the robot
    // reaches that point
    init_time_count++;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    vec_goal = transformToMap * vec_init;
    wp.point.x = vec_goal.x();
    wp.point.y = vec_goal.y();
    wp.point.z = vec_goal.z();
    waypoint_pub.publish(wp);
    double dist = sqrt((wp.point.x - current_odom_x) * (wp.point.x - current_odom_x) +
                       (wp.point.y - current_odom_y) * (wp.point.y - current_odom_y));
    double dist_to_home = sqrt((home_point.x - current_odom_x) * (home_point.x - current_odom_x) +
                               (home_point.y - current_odom_y) * (home_point.y - current_odom_y));
    if (dist < 0.5 && dist_to_home > 0.5)
      wp_ongoing = false;
    if (init_time_count >= init_time / 0.1 && dist_to_home > 0.5)
      wp_ongoing = false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");
  
  nhPrivate.getParam("simulation", simulation);
  nhPrivate.getParam("/interface/dtime", dtime);
  nhPrivate.getParam("/interface/initX", init_x);
  nhPrivate.getParam("/interface/initY", init_y);
  nhPrivate.getParam("/interface/initZ", init_z);
  nhPrivate.getParam("/interface/initTime", init_time);
  nhPrivate.getParam("/interface/returnHomeThres", return_home_threshold);
  nhPrivate.getParam("/interface/robotMovingThres", robot_moving_threshold);
  nhPrivate.getParam("/interface/tfFrame", map_frame);
  nhPrivate.getParam("/interface/autoExp", begin_signal);
  nhPrivate.getParam("/interface/waypointTopic", waypoint_topic);
  nhPrivate.getParam("/interface/cmdVelTopic", cmd_vel_topic);
  nhPrivate.getParam("/interface/graphPlannerCommandTopic", gp_command_topic);
  nhPrivate.getParam("/interface/effectivePlanTimeTopic", effective_plan_time_topic);
  nhPrivate.getParam("/interface/totalPlanTimeTopic", total_plan_time_topic);
  nhPrivate.getParam("/interface/gpStatusTopic", gp_status_topic);
  nhPrivate.getParam("/interface/odomTopic", odom_topic);
  nhPrivate.getParam("/interface/beginSignalTopic", begin_signal_topic);
  nhPrivate.getParam("/interface/stopSignalTopic", stop_signal_topic);

  waypoint_pub = nh.advertise<geometry_msgs::PointStamped>(waypoint_topic, 5);
  gp_command_pub = nh.advertise<graph_planner::GraphPlannerCommand>(gp_command_topic, 1);
  effective_plan_time_pub = nh.advertise<std_msgs::Float32>(effective_plan_time_topic, 1);
  total_plan_time_pub = nh.advertise<std_msgs::Float32>(total_plan_time_topic, 1);
  gp_status_sub = nh.subscribe<graph_planner::GraphPlannerStatus>(gp_status_topic, 1, gp_status_callback);
  waypoint_sub = nh.subscribe<geometry_msgs::PointStamped>(waypoint_topic, 1, waypoint_callback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, odom_callback);
  begin_signal_sub = nh.subscribe<std_msgs::Bool>(begin_signal_topic, 1, begin_signal_callback);
  stop_signal_pub = nh.advertise<std_msgs::Bool>(stop_signal_topic, 1);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  while (!begin_signal)
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ROS_INFO("Waiting for Odometry");
  }

  ROS_INFO("Starting the planner: Performing initialization motion");
  initilization();
  ros::Duration(1.0).sleep();

  std::cout << std::endl << "\033[1;32mExploration Started\033[0m\n" << std::endl;
  total_time.data = 0;
  plan_start = steady_clock::now();
  // Start planning: The planner is called and the computed goal point sent to
  // the graph planner.
  int iteration = 0;
  while (ros::ok())
  {
    if (!return_home)
    {
      if (iteration != 0)
      {
        for (int i = 0; i < 8; i++)
        {
          printf(cursup);
          printf(cursclean);
        }
      }
      std::cout << "Planning iteration " << iteration << std::endl;
      dsvplanner::dsvplanner_srv planSrv;
      dsvplanner::clean_frontier_srv cleanSrv;
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

        if (planSrv.response.mode.data == 2)
        {
          return_home = true;
          goal_point = home_point;
          std::cout << std::endl << "\033[1;32mExploration completed, returning home\033[0m" << std::endl << std::endl;
          effective_time.data = 0;
          effective_plan_time_pub.publish(effective_time);
        }
        else
        {
          return_home = false;
          goal_point = planSrv.response.goal[0];
          plan_over = steady_clock::now();
          time_span = plan_over - plan_start;
          effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
          effective_plan_time_pub.publish(effective_time);
        }
        total_time.data += effective_time.data;
        total_plan_time_pub.publish(total_time);

        if (!simulation)
        {  // when not in simulation mode, the robot will go to
           // the goal point according to graph planner
          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub.publish(graph_planner_command);
          ros::Duration(dtime).sleep();  // give sometime to graph planner for
                                         // searching path to goal point
          ros::spinOnce();               // update gp_in_progree
          int count = 200;
          previous_odom_x = current_odom_x;
          previous_odom_y = current_odom_y;
          previous_odom_z = current_odom_z;
          while (gp_in_progress)
          {                              // if the waypoint keep the same for 20
                                         // (200*0.1)
            ros::Duration(0.1).sleep();  // seconds, then give up the goal
            wayPoint_pre = wayPoint;
            ros::spinOnce();
            bool robotMoving = robotPositionChange();
            if (robotMoving)
            {
              count = 200;
            }
            else
            {
              count--;
            }
            if (count <= 0)
            {  // when the goal point cannot be reached, clean
               // its correspoinding frontier if there is
              cleanSrv.request.header.stamp = ros::Time::now();
              cleanSrv.request.header.frame_id = map_frame;
              ros::service::call("cleanFrontierSrv", cleanSrv);
              ros::Duration(0.1).sleep();
              break;
            }
          }

          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_DISABLE;
          gp_command_pub.publish(graph_planner_command);
        }
        else
        {  // simulation mode is used when testing this planning algorithm
           // with bagfiles where robot will
          // not move to the planned goal. When in simulation mode, robot will
          // keep replanning every two seconds
          for (size_t i = 0; i < planSrv.response.goal.size(); i++)
          {
            graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = planSrv.response.goal[i];
            gp_command_pub.publish(graph_planner_command);
            ros::Duration(2).sleep();
            break;
          }
        }
        plan_start = steady_clock::now();
      }
      else
      {
        std::cout << "Cannot call drrt planner." << std::flush;

        ros::Duration(1.0).sleep();
      }
      iteration++;
    }
    else
    {
      ros::spinOnce();
      if (fabs(current_odom_x - home_point.x) + fabs(current_odom_y - home_point.y) +
              fabs(current_odom_z - home_point.z) <=
          return_home_threshold)
      {
        printf(cursclean);
        std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
        printf(cursup);
        std_msgs::Bool stop_exploring;
        stop_exploring.data = true;
        stop_signal_pub.publish(stop_exploring);
      }
      else
      {
        while (!gp_in_progress)
        {
          ros::spinOnce();
          ros::Duration(2.0).sleep();

          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub.publish(graph_planner_command);
        }
      }
      ros::Duration(0.1).sleep();
    }
  }
}
