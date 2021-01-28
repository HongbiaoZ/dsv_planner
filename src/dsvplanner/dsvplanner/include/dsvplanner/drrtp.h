/**************************************************************************
drrtp.h
Header of the drrtp (dynamic_rrtplanner) class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef DRRTP_H_
#define DRRTP_H_

#include "octomap_world/octomap_manager.h"
#include "dsvplanner/dual_state_graph.h"
#include "dsvplanner/dual_state_frontier.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/drrt_base.h"
#include "dsvplanner/drrt.h"

namespace dsvplanner_ns
{
class drrtPlanner
{
public:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber odomSub_;

  ros::ServiceServer plannerService_;
  ros::ServiceServer cleanFrontierService_;

  Params params_;
  volumetric_mapping::OctomapManager* manager_;
  DualStateGraph* dual_state_graph_;
  DualStateFrontier* dual_state_frontier_;
  Drrt* drrt_;

  bool init();
  bool setParams();
  // bool setPublisherPointer();
  void odomCallback(const nav_msgs::Odometry& pose);
  bool plannerServiceCallback(dsvplanner::dsvplanner_srv::Request& req, dsvplanner::dsvplanner_srv::Response& res);
  bool cleanFrontierServiceCallback(dsvplanner::clean_frontier_srv::Request& req,
                                    dsvplanner::clean_frontier_srv::Response& res);
  void cleanLastSelectedGlobalFrontier();

  drrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~drrtPlanner();

private:
  std::string odomSubTopic;
  std::string terrainCloudSubTopic;
  std::string terrainVoxelElevSubTopic;
  std::string newTreePathPubTopic;
  std::string remainingTreePathPubTopic;
  std::string boundaryPubTopic;
  std::string globalSelectedFrontierPubTopic;
  std::string localSelectedFrontierPubTopic;
  std::string plantimePubTopic;
  std::string nextGoalPubTopic;
  std::string pointInSensorRangePubTopic;
  std::string terrainNoGroundPubTopic;
  std::string shutDownTopic;
  std::string plannerServiceName;
  std::string cleanFrontierServiceName;

  std::chrono::steady_clock::time_point plan_start_;
  std::chrono::steady_clock::time_point RRT_generate_over_;
  std::chrono::steady_clock::time_point gain_computation_over_;
  std::chrono::steady_clock::time_point plan_over_;
  std::chrono::steady_clock::duration time_span;
};
}

#endif  // DRRTP_H
