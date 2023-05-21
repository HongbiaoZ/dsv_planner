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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pointStamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/pointCloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>

#include "dsvplanner/srv/clean_frontier_srv.hpp"
#include "dsvplanner/srv/dsvplanner_srv.hpp"
#include "graph_planner/msg/graph_planner_command.hpp"
#include "graph_planner/msg/graph_planner_status.hpp"

using namespace std::chrono;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

geometry_msgs::msg::Point wayPoint;
geometry_msgs::msg::Point wayPoint_pre;
geometry_msgs::msg::Point goal_point;
geometry_msgs::msg::Point home_point;
graph_planner::msg::GraphPlannerCommand graph_planner_command;
std_msgs::msg::Float32 effective_time;
std_msgs::msg::Float32 total_time;

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

rclcpp::Node::SharedPtr nh = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub;
rclcpp::Publisher<graph_planner::msg::GraphPlannerCommand>::SharedPtr gp_command_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr effective_plan_time_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr total_plan_time_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_pub;

rclcpp::Subscription<graph_planner::msg::GraphPlannerStatus>::SharedPtr gp_status_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_signal_sub;

void gp_status_callback(const graph_planner::msg::GraphPlannerStatus::SharedPtr msg)
{
  if (msg->status == graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS)
    gp_in_progress = true;
  else
  {
    gp_in_progress = false;
  }
}

void waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  wayPoint = msg->point;
  wp_state = true;
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_x = msg->pose.pose.position.x;
  current_odom_y = msg->pose.pose.position.y;
  current_odom_z = msg->pose.pose.position.z;

  transformToMap.setOrigin(
      tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

void begin_signal_callback(const std_msgs::msg::Bool::SharedPtr msg)
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
  geometry_msgs::msg::PointStamped wp;
  wp.header.frame_id = map_frame;
  wp.header.stamp = nh->now();
  wp.point.x = vec_goal.x();
  wp.point.y = vec_goal.y();
  wp.point.z = vec_goal.z();
  home_point.x = current_odom_x;
  home_point.y = current_odom_y;
  home_point.z = current_odom_z;

  rclcpp::sleep_for(rclcpp::Duration(0.5).nanoseconds());  // wait for sometime to make sure waypoint can be
                               // published properly

  waypoint_pub->publish(wp);
  bool wp_ongoing = true;
  int init_time_count = 0;
  while (wp_ongoing)
  {  // Keep publishing initial waypoint until the robot
    // reaches that point
    init_time_count++;
    rclcpp::sleep_for(rclcpp::Duration(0.1).nanoseconds());
    rclcpp::spin_some(nh);
    vec_goal = transformToMap * vec_init;
    wp.point.x = vec_goal.x();
    wp.point.y = vec_goal.y();
    wp.point.z = vec_goal.z();
    waypoint_pub->publish(wp);
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
  ros::init(argc, argv);
  nh = rclcpp::Node::make_shared("exploration");
  
  nh->declare_parameter("/interface/simulation");
  nh->declare_parameter("/interface/dtime");
  nh->declare_parameter("/interface/initX");
  nh->declare_parameter("/interface/initY");
  nh->declare_parameter("/interface/initZ");
  nh->declare_parameter("/interface/initTime");
  nh->declare_parameter("/interface/returnHomeThres");
  nh->declare_parameter("/interface/robotMovingThres");
  nh->declare_parameter("/interface/tfFrame");
  nh->declare_parameter("/interface/autoExp");
  nh->declare_parameter("/interface/waypointTopic");
  nh->declare_parameter("/interface/cmdVelTopic");
  nh->declare_parameter("/interface/graphPlannerCommandTopic");
  nh->declare_parameter("/interface/effectivePlanTimeTopic");
  nh->declare_parameter("/interface/totalPlanTimeTopic");
  nh->declare_parameter("/interface/gpStatusTopic");
  nh->declare_parameter("/interface/odomTopic");
  nh->declare_parameter("/interface/beginSignalTopic");
  nh->declare_parameter("/interface/stopSignalTopic");

  nh->get_parameter("/interface/simulation", simulation);
  nh->get_parameter("/interface/dtime", dtime);
  nh->get_parameter("/interface/initX", init_x);
  nh->get_parameter("/interface/initY", init_y);
  nh->get_parameter("/interface/initZ", init_z);
  nh->get_parameter("/interface/initTime", init_time);
  nh->get_parameter("/interface/returnHomeThres", return_home_threshold);
  nh->get_parameter("/interface/robotMovingThres", robot_moving_threshold);
  nh->get_parameter("/interface/tfFrame", map_frame);
  nh->get_parameter("/interface/autoExp", begin_signal);
  nh->get_parameter("/interface/waypointTopic", waypoint_topic);
  nh->get_parameter("/interface/cmdVelTopic", cmd_vel_topic);
  nh->get_parameter("/interface/graphPlannerCommandTopic", gp_command_topic);
  nh->get_parameter("/interface/effectivePlanTimeTopic", effective_plan_time_topic);
  nh->get_parameter("/interface/totalPlanTimeTopic", total_plan_time_topic);
  nh->get_parameter("/interface/gpStatusTopic", gp_status_topic);
  nh->get_parameter("/interface/odomTopic", odom_topic);
  nh->get_parameter("/interface/beginSignalTopic", begin_signal_topic);
  nh->get_parameter("/interface/stopSignalTopic", stop_signal_topic);

  waypoint_pub = nh->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 5);
  gp_command_pub = nh->create_publisher<graph_planner::msg::GraphPlannerCommand>(gp_command_topic, 1);
  effective_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(effective_plan_time_topic, 1);
  total_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(total_plan_time_topic, 1);
  stop_signal_pub = nh->create_publisher<std_msgs::msg::Bool>(stop_signal_topic, 1);

  gp_status_sub = nh_->create_subscription<graph_planner::msg::GraphPlannerStatus>(gp_status_topic, qos, gp_status_callback);
  waypoint_sub = nh_->create_subscription<geometry_msgs::msg::PointStamped>(waypoint_topic, qos, waypoint_callback);
  odom_sub = nh_->create_subscription<nav_msgs::msg::Odometry>(odom_topic, qos, odom_callback);
  begin_signal_sub = nh_->create_subscription<std_msgs::msg::Bool>(begin_signal_topic, qos, begin_signal_callback);

  rclcpp::Client<dsvplanner::srv::clean_frontier_srv>::SharedPtr frontier_cleaner_client =
    nh->create_client<dsvplanner::srv::clean_frontier_srv>("cleanFrontierSrv");
  rclcpp::Client<dsvplanner::srv::dsvplanner_srv>::SharedPtr drrt_planner_client =
    nh->create_client<dsvplanner::srv::dsvplanner_srv>("drrtPlannerSrv");

  rclcpp::sleep_for(rclcpp::Duration(1).nanoseconds());
  rclcpp::spin_some(nh);

  while (!begin_signal)
  {
    rclcpp::sleep_for(rclcpp::Duration(0.5).nanoseconds());
    rclcpp::spin_some(nh);
    ROS_INFO("Waiting for Odometry");
  }

  ROS_INFO("Starting the planner: Performing initialization motion");
  initilization();
  rclcpp::sleep_for(rclcpp::Duration(1).nanoseconds());

  std::cout << std::endl << "\033[1;32mExploration Started\033[0m\n" << std::endl;
  total_time.data = 0;
  plan_start = steady_clock::now();
  // Start planning: The planner is called and the computed goal point sent to
  // the graph planner.
  int iteration = 0;
  while (rclcpp::ok())
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
      auto planSrv = std::make_shared<dsvplanner::srv::dsvplanner_srv::Request>();
      auto cleanSrv = std::make_shared<dsvplanner::srv::clean_frontier_srv::Request>();
      planSrv->header.stamp = nh->now();
      planSrv->header.seq = iteration;
      planSrv->header.frame_id = map_frame;
      auto response = drrt_planner_client->async_send_request(planSrv);
      if (rclcpp::spin_until_future_complete(nh, result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        if (response.get()->goal.size() == 0)
        {  // usually the size should be 1 if planning successfully
          rclcpp::sleep_for(rclcpp::Duration(1).nanoseconds());
          continue;
        }

        if (response.get()->mode.data == 2)
        {
          return_home = true;
          goal_point = home_point;
          std::cout << std::endl << "\033[1;32mExploration completed, returning home\033[0m" << std::endl << std::endl;
          effective_time.data = 0;
          effective_plan_time_pub->publish(effective_time);
        }
        else
        {
          return_home = false;
          goal_point = response.get()->goal[0];
          plan_over = steady_clock::now();
          time_span = plan_over - plan_start;
          effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
          effective_plan_time_pub->publish(effective_time);
        }
        total_time.data += effective_time.data;
        total_plan_time_pub->publish(total_time);

        if (!simulation)
        {  // when not in simulation mode, the robot will go to
           // the goal point according to graph planner
          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub->publish(graph_planner_command);
          rclcpp::sleep_for(rclcpp::Duration(dtime).nanoseconds());  // give sometime to graph planner for
                                         // searching path to goal point
          rclcpp::spin_some(nh);              // update gp_in_progree
          int count = 200;
          previous_odom_x = current_odom_x;
          previous_odom_y = current_odom_y;
          previous_odom_z = current_odom_z;
          while (gp_in_progress)
          {                              // if the waypoint keep the same for 20
                                         // (200*0.1)
            rclcpp::sleep_for(rclcpp::Duration(0.1).nanoseconds());  // seconds, then give up the goal
            wayPoint_pre = wayPoint;
            rclcpp::spin_some(nh);
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
              cleanSrv->header.stamp = nh->now();
              cleanSrv->header.frame_id = map_frame;
              auto response = frontier_cleaner_client->async_send_request(cleanSrv);
              rclcpp::sleep_for(rclcpp::Duration(0.1).nanoseconds());
              break;
            }
          }

          graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_DISABLE;
          gp_command_pub->publish(graph_planner_command);
        }
        else
        {  // simulation mode is used when testing this planning algorithm
           // with bagfiles where robot will
          // not move to the planned goal. When in simulation mode, robot will
          // keep replanning every two seconds
          for (size_t i = 0; i < response.get()->goal.size(); i++)
          {
            graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = response.get()->goal[i];
            gp_command_pub->publish(graph_planner_command);
            rclcpp::sleep_for(rclcpp::Duration(2.0).nanoseconds());
            break;
          }
        }
        plan_start = steady_clock::now();
      }
      else
      {
        std::cout << "Cannot call drrt planner." << std::flush;

        rclcpp::sleep_for(rclcpp::Duration(1).nanoseconds());
      }
      iteration++;
    }
    else
    {
      rclcpp::spin_some(nh);
      if (fabs(current_odom_x - home_point.x) + fabs(current_odom_y - home_point.y) +
              fabs(current_odom_z - home_point.z) <=
          return_home_threshold)
      {
        printf(cursclean);
        std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
        printf(cursup);
        std_msgs::Bool stop_exploring;
        stop_exploring.data = true;
        stop_signal_pub->publish(stop_exploring);
      }
      else
      {
        while (!gp_in_progress)
        {
          rclcpp::spin_some(nh);
          rclcpp::sleep_for(rclcpp::Duration(2.0).nanoseconds());

          graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub->publish(graph_planner_command);
        }
      }
      rclcpp::sleep_for(rclcpp::Duration(0.1).nanoseconds());
    }
  }
}
