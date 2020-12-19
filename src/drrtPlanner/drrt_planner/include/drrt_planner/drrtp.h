/**************************************************************************
drrtp.h
Header of the drrt_planner (dynamic_rrt_based_planner) class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef DRRTP_H_
#define DRRTP_H_

#include "octomap_world/octomap_manager.h"
#include "drrt_planner/dual_state_graph.h"
#include "drrt_planner/dual_state_frontier.h"
#include "drrt_planner/drrt_planner_srv.h"
#include "drrt_planner/clean_frontier_srv.h"
#include "drrt_planner/drrt_base.h"
#include "drrt_planner/drrt.h"

namespace drrt_planner_ns
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
  bool plannerServiceCallback(drrt_planner::drrt_planner_srv::Request& req,
                              drrt_planner::drrt_planner_srv::Response& res);
  bool cleanFrontierServiceCallback(drrt_planner::clean_frontier_srv::Request& req,
                                    drrt_planner::clean_frontier_srv::Response& res);
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
};
}

#endif  // DRRTP_H
