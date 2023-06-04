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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

#include "dsvplanner/srv/clean_frontier.hpp"
#include "dsvplanner/srv/dsvplanner.hpp"
#include "graph_planner/msg/graph_planner_command.hpp"
#include "graph_planner/msg/graph_planner_status.hpp"

using namespace std::chrono;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"
#define UNIT 1000000000

geometry_msgs::msg::Point wayPoint;
geometry_msgs::msg::Point wayPoint_pre;
geometry_msgs::msg::Point goal_point;
geometry_msgs::msg::Point home_point;
graph_planner::msg::GraphPlannerCommand graph_planner_command;
std_msgs::msg::Float32 effective_time;
std_msgs::msg::Float32 total_time;

bool play_bag = false;    // control whether use graph planner to follow path
bool begin_signal = false;  // trigger the planner
bool gp_in_progress = false;
bool wp_state = false;
bool return_home = false;
bool odom_is_ready = false;
bool call_dsvp_successfully = true;
bool initialized = false;
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
std::string gp_command_topic = "/graph_planner_command";
std::string effective_plan_time_topic = "/runtime";
std::string total_plan_time_topic = "/totaltime";
std::string gp_status_topic = "/graph_planner_status";
std::string odom_topic = "/state_estimation";
std::string begin_signal_topic = "/start_exploring";
std::string stop_signal_topic = "/stop_exploring";

tf2::Transform transformToMap;

steady_clock::time_point plan_start;
steady_clock::time_point plan_over;
steady_clock::duration time_span;

rclcpp::Node::SharedPtr nh = nullptr;
rclcpp::Node::SharedPtr nh2 = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub;
rclcpp::Publisher<graph_planner::msg::GraphPlannerCommand>::SharedPtr gp_command_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr effective_plan_time_pub;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr total_plan_time_pub;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_signal_pub;

rclcpp::Subscription<graph_planner::msg::GraphPlannerStatus>::SharedPtr gp_status_sub;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr begin_signal_sub;

rclcpp::Client<dsvplanner::srv::CleanFrontier>::SharedPtr frontier_cleaner_client;
rclcpp::Client<dsvplanner::srv::Dsvplanner>::SharedPtr drrt_planner_client;

void gp_status_callback(const graph_planner::msg::GraphPlannerStatus::SharedPtr msg)
{
  if (msg->status == graph_planner::msg::GraphPlannerStatus::STATUS_IN_PROGRESS)
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
      tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  transformToMap.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  odom_is_ready = true;
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

void execute(){
  static int iteration = 0;
  if(initialized){
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
      if(call_dsvp_successfully){
        std::cout << "Planning iteration " << iteration << std::endl;
        auto planSrv = std::make_shared<dsvplanner::srv::Dsvplanner::Request>();
        auto cleanSrv = std::make_shared<dsvplanner::srv::CleanFrontier::Request>();
        planSrv->header.stamp = nh->now();

        call_dsvp_successfully = false;
        auto callback = [&](rclcpp::Client<dsvplanner::srv::Dsvplanner>::SharedFuture future) { 
          auto result = future.get();

          plan_over = steady_clock::now();
          time_span = plan_over - plan_start;
          effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;

          if (result->goal.size() == 0)
          {  // usually the size should be 1 if planning successfully
            effective_plan_time_pub->publish(effective_time);
          }else{
            if (result->mode == 2)
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
              goal_point = result->goal[0];
              // time_span = plan_over - plan_start;
              // effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
              effective_plan_time_pub->publish(effective_time);
            }
            total_time.data += effective_time.data;
            total_plan_time_pub->publish(total_time);
            if (!play_bag)
            {  // when not in simulation mode, the robot will go to
              // the goal point according to graph planner
              graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
              graph_planner_command.location = goal_point;
              gp_command_pub->publish(graph_planner_command);
              sleep(1);  // give sometime to graph planner for
                                            // searching path to goal point
              // exector.spin_once();              // update gp_in_progree
              int count = 200;
              previous_odom_x = current_odom_x;
              previous_odom_y = current_odom_y;
              previous_odom_z = current_odom_z;
              while (gp_in_progress)
              {                              // if the waypoint keep the same for 20
                // std::cout<<"gp_in_progress "<<gp_in_progress<<std::endl;                            // (200*0.1)
                usleep(100000);  // seconds, then give up the goal
                // exector.spin_once();
                wayPoint_pre = wayPoint;
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

                  usleep(100000);
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
              for (size_t i = 0; i < result->goal.size(); i++)
              {
                graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                graph_planner_command.location = result->goal[i];
                gp_command_pub->publish(graph_planner_command);
                while(1){}
                break;
              }
            }
            plan_start = steady_clock::now();
          }
          call_dsvp_successfully = true;
        // Use result->responses[] and do something 
        };
        auto result_future = drrt_planner_client->async_send_request(planSrv, callback);
        iteration++;
      }
    }
    else
    {
      if (fabs(current_odom_x - home_point.x) + fabs(current_odom_y - home_point.y) +
              fabs(current_odom_z - home_point.z) <=
          return_home_threshold)
      {
        printf(cursclean);
        std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
        printf(cursup);
        std_msgs::msg::Bool stop_exploring;
        stop_exploring.data = true;
        stop_signal_pub->publish(stop_exploring);
      }
      else
      {
        while (!gp_in_progress)
        {
          sleep(2);
          // executor.spin_some();
          graph_planner_command.command = graph_planner::msg::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub->publish(graph_planner_command);
        }
      }
      usleep(100000);
    }
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  nh = rclcpp::Node::make_shared("exploration");
  executor.add_node(nh);
  
  nh->declare_parameter("interface/replay", play_bag);
  nh->declare_parameter("interface/dtime", dtime);
  nh->declare_parameter("interface/initX", init_x);
  nh->declare_parameter("interface/initY", init_y);
  nh->declare_parameter("interface/initZ", init_z);
  nh->declare_parameter("interface/initTime", init_time);
  nh->declare_parameter("interface/returnHomeThres", return_home_threshold);
  nh->declare_parameter("interface/robotMovingThres", robot_moving_threshold);
  nh->declare_parameter("interface/tfFrame", map_frame);
  nh->declare_parameter("interface/autoExp", begin_signal);
  nh->declare_parameter("interface/waypointTopic", waypoint_topic);
  nh->declare_parameter("interface/graphPlannerCommandTopic", gp_command_topic);
  nh->declare_parameter("interface/effectivePlanTimeTopic", effective_plan_time_topic);
  nh->declare_parameter("interface/totalPlanTimeTopic", total_plan_time_topic);
  nh->declare_parameter("interface/gpStatusTopic", gp_status_topic);
  nh->declare_parameter("interface/odomTopic", odom_topic);
  nh->declare_parameter("interface/beginSignalTopic", begin_signal_topic);
  nh->declare_parameter("interface/stopSignalTopic", stop_signal_topic);

  nh->get_parameter("interface/replay", play_bag);
  nh->get_parameter("interface/dtime", dtime);
  nh->get_parameter("interface/initX", init_x);
  nh->get_parameter("interface/initY", init_y);
  nh->get_parameter("interface/initZ", init_z);
  nh->get_parameter("interface/initTime", init_time);
  nh->get_parameter("interface/returnHomeThres", return_home_threshold);
  nh->get_parameter("interface/robotMovingThres", robot_moving_threshold);
  nh->get_parameter("interface/tfFrame", map_frame);
  nh->get_parameter("interface/autoExp", begin_signal);
  nh->get_parameter("interface/waypointTopic", waypoint_topic);
  nh->get_parameter("interface/graphPlannerCommandTopic", gp_command_topic);
  nh->get_parameter("interface/effectivePlanTimeTopic", effective_plan_time_topic);
  nh->get_parameter("interface/totalPlanTimeTopic", total_plan_time_topic);
  nh->get_parameter("interface/gpStatusTopic", gp_status_topic);
  nh->get_parameter("interface/odomTopic", odom_topic);
  nh->get_parameter("interface/beginSignalTopic", begin_signal_topic);
  nh->get_parameter("interface/stopSignalTopic", stop_signal_topic);

  rclcpp::CallbackGroup::SharedPtr client_cb_group_ =  nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_ =  nh->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = sub_cb_group_;

  waypoint_pub = nh->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 5);
  gp_command_pub = nh->create_publisher<graph_planner::msg::GraphPlannerCommand>(gp_command_topic, 2);
  effective_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(effective_plan_time_topic, 2);
  total_plan_time_pub = nh->create_publisher<std_msgs::msg::Float32>(total_plan_time_topic, 2);
  stop_signal_pub = nh->create_publisher<std_msgs::msg::Bool>(stop_signal_topic, 2);

  gp_status_sub = nh->create_subscription<graph_planner::msg::GraphPlannerStatus>(gp_status_topic, 1, gp_status_callback, options);
  waypoint_sub = nh->create_subscription<geometry_msgs::msg::PointStamped>(waypoint_topic, 5, waypoint_callback, options);
  odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 5, odom_callback, options);
  begin_signal_sub = nh->create_subscription<std_msgs::msg::Bool>(begin_signal_topic, 5, begin_signal_callback, options);

  frontier_cleaner_client =
    nh->create_client<dsvplanner::srv::CleanFrontier>("cleanFrontierSrv", rmw_qos_profile_services_default, client_cb_group_);
  drrt_planner_client =
    nh->create_client<dsvplanner::srv::Dsvplanner>("drrtPlannerSrv", rmw_qos_profile_services_default, client_cb_group_);

  rclcpp::TimerBase::SharedPtr executeTimer_ = nh->create_wall_timer(0.1s, execute, sub_cb_group_);

  sleep(1);
  executor.spin_some();

  while (!begin_signal || !odom_is_ready)
  {
    usleep(500000);
    executor.spin_some();
    RCLCPP_INFO(nh->get_logger(), "Waiting for Odometry");
  }

  RCLCPP_INFO(nh->get_logger(), "Starting the planner: Performing initialization motion");
  {
    tf2::Vector3 vec_init(init_x, init_y, init_z);
    tf2::Vector3 vec_goal;
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

    usleep(500000); // wait for sometime to make sure waypoint can be
                                // published properly

    waypoint_pub->publish(wp);
    bool wp_ongoing = true;
    int init_time_count = 0;
    while (wp_ongoing)
    {  // Keep publishing initial waypoint until the robot
      // reaches that point
      init_time_count++;
      usleep(100000);
      executor.spin_some();
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
  // initilization();
  sleep(1);
  initialized = true;

  std::cout << std::endl << "\033[1;32mExploration Started\033[0m\n" << std::endl;
  total_time.data = 0;
  plan_start = steady_clock::now();
  executor.spin();
}
