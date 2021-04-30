/*
drrtp.cpp
Implementation of drrt_planner class

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>

#include <visualization_msgs/Marker.h>

#include <dsvplanner/drrtp.h>

using namespace Eigen;

dsvplanner_ns::drrtPlanner::drrtPlanner(const ros::NodeHandle &nh,
                                        const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);
  dual_state_graph_ = new DualStateGraph(nh_, nh_private_, manager_);
  dual_state_frontier_ = new DualStateFrontier(nh_, nh_private_, manager_);
  drrt_ = new Drrt(manager_, dual_state_graph_, dual_state_frontier_);

  init();
}

dsvplanner_ns::drrtPlanner::~drrtPlanner() {
  if (manager_) {
    delete manager_;
  }
  if (dual_state_graph_) {
    delete dual_state_graph_;
  }
  if (drrt_) {
    delete drrt_;
  }
}

void dsvplanner_ns::drrtPlanner::odomCallback(const nav_msgs::Odometry &pose) {
  drrt_->setRootWithOdom(pose);
  // Planner is now ready to plan.
  drrt_->plannerReady_ = true;
}

bool dsvplanner_ns::drrtPlanner::plannerServiceCallback(
    dsvplanner::dsvplanner_srv::Request &req,
    dsvplanner::dsvplanner_srv::Response &res) {
  plan_start_ = std::chrono::steady_clock::now();
  // drrt_->gotoxy(0, 10);  // Go to the specific line on the screen
  // Check if the planner is ready.
  if (!drrt_->plannerReady_) {
    std::cout << "No odometry. Planner is not ready!" << std::endl;
    return true;
  }
  if (manager_ == NULL) {
    std::cout << "No octomap. Planner is not ready!" << std::endl;
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0) {
    std::cout << "Octomap is empty. Planner is not set up!" << std::endl;
    return true;
  }

  // set terrain points and terrain voxel elevation
  drrt_->setTerrainVoxelElev();

  // Clear old tree and the last global frontier.
  cleanLastSelectedGlobalFrontier();
  drrt_->clear();

  // Reinitialize.
  drrt_->plannerInit();

  // Iterate the tree construction method.
  int loopCount = 0;
  while (ros::ok() && drrt_->remainingFrontier_ &&
         drrt_->getNodeCounter() < params_.kCuttoffIterations &&
         (!(drrt_->getNodeCounter() >= params_.kVertexSize &&
            drrt_->gainFound()))) {
    if (loopCount > 1000 * (drrt_->getNodeCounter() + 1)) {
      // ROS_INFO_THROTTLE(1, "Exceeding maximum failed iterations, give up the
      // current planning!");
      break;
    }
    drrt_->plannerIterate();
    loopCount++;
  }

  // Publish rrt
  drrt_->publishNode();
  std::cout << "     New node number is " << drrt_->getNodeCounter() << "\n"
            << "     Current local RRT size is "
            << dual_state_graph_->getLocalVertexSize() << "\n"
            << "     Current global graph size is "
            << dual_state_graph_->getGlobalVertexSize() << std::endl;
  RRT_generate_over_ = std::chrono::steady_clock::now();
  time_span = RRT_generate_over_ - plan_start_;
  double rrtGenerateTime = double(time_span.count()) *
                           std::chrono::steady_clock::period::num /
                           std::chrono::steady_clock::period::den;
  // Reset planner state
  drrt_->global_plan_pre_ = drrt_->global_plan_;
  drrt_->global_plan_ = false;
  drrt_->local_plan_ = false;

  // update planner state of last iteration
  dual_state_frontier_->setPlannerStatus(drrt_->global_plan_pre_);

  // Update planner state of next iteration
  geometry_msgs::Point robot_position;
  robot_position.x = drrt_->root_[0];
  robot_position.y = drrt_->root_[1];
  robot_position.z = drrt_->root_[2];
  if (!drrt_->nextNodeFound_ && drrt_->global_plan_pre_ &&
      drrt_->gainFound() <= 0) {
    drrt_->return_home_ = true;
    geometry_msgs::Point home_position;
    home_position.x = 0;
    home_position.y = 0;
    home_position.z = 0;
    res.goal.push_back(home_position);
    res.mode.data = 2; // mode 2 means returning home

    dual_state_frontier_->cleanAllUselessFrontiers();
    return true;
  } else if (!drrt_->nextNodeFound_ && !drrt_->global_plan_pre_ &&
             dual_state_graph_->getGain(robot_position) <= 0) {
    drrt_->global_plan_ = true;
    std::cout << "     No Remaining local frontiers  "
              << "\n"
              << "     Switch to relocation stage "
              << "\n"
              << "     Total plan lasted " << 0 << std::endl;
    return true;
  } else {
    drrt_->local_plan_ = true;
  }
  gain_computation_over_ = std::chrono::steady_clock::now();
  time_span = gain_computation_over_ - RRT_generate_over_;
  double getGainTime = double(time_span.count()) *
                       std::chrono::steady_clock::period::num /
                       std::chrono::steady_clock::period::den;

  // Extract next goal.
  geometry_msgs::Point next_goal_position;
  if (drrt_->nextNodeFound_) {
    dual_state_graph_->best_vertex_id_ = drrt_->NextBestNodeIdx_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestGlobalVertexPosition();
  } else if (drrt_->global_plan_pre_ == true && drrt_->gainFound()) {
    dual_state_graph_->best_vertex_id_ = drrt_->bestNodeId_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  } else {
    dual_state_graph_->updateGlobalGraph();
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  dual_state_graph_->setCurrentPlannerStatus(drrt_->global_plan_pre_);
  res.goal.push_back(next_goal_position);
  res.mode.data = 1; // mode 1 means exploration

  geometry_msgs::PointStamped next_goal_point;
  next_goal_point.header.frame_id = "/map";
  next_goal_point.point = next_goal_position;
  params_.nextGoalPub_.publish(next_goal_point);

  plan_over_ = std::chrono::steady_clock::now();
  time_span = plan_over_ - plan_start_;
  double plantime = double(time_span.count()) *
                    std::chrono::steady_clock::period::num /
                    std::chrono::steady_clock::period::den;
  std::cout << "     RRT generation lasted  " << rrtGenerateTime << "\n"
            << "     Computiong gain lasted " << getGainTime << "\n"
            << "     Total plan lasted " << plantime << std::endl;
  return true;
}

bool dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback(
    dsvplanner::clean_frontier_srv::Request &req,
    dsvplanner::clean_frontier_srv::Response &res) {
  if (drrt_->nextNodeFound_) {
    dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
    dual_state_frontier_->gloabalFrontierUpdate();
  } else {
    dual_state_graph_->clearLocalGraph();
  }
  res.success = true;

  return true;
}

void dsvplanner_ns::drrtPlanner::cleanLastSelectedGlobalFrontier() {
  // only when last plan is global plan, this function will be executed to clear
  // last selected global
  // frontier.
  if (drrt_->nextNodeFound_) {
    dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
    dual_state_frontier_->gloabalFrontierUpdate();
  }
}

bool dsvplanner_ns::drrtPlanner::setParams() {
  nh_private_.getParam("/rm/kSensorPitch", params_.sensorPitch);
  nh_private_.getParam("/rm/kSensorHorizontal", params_.sensorHorizontalView);
  nh_private_.getParam("/rm/kSensorVertical", params_.sensorVerticalView);
  nh_private_.getParam("/rm/kVehicleHeight", params_.kVehicleHeight);
  nh_private_.getParam("/rm/kBoundX", params_.boundingBox[0]);
  nh_private_.getParam("/rm/kBoundY", params_.boundingBox[1]);
  nh_private_.getParam("/rm/kBoundZ", params_.boundingBox[2]);
  nh_private_.getParam("/drrt/gain/kFree", params_.kGainFree);
  nh_private_.getParam("/drrt/gain/kOccupied", params_.kGainOccupied);
  nh_private_.getParam("/drrt/gain/kUnknown", params_.kGainUnknown);
  nh_private_.getParam("/drrt/gain/kRange", params_.kGainRange);
  nh_private_.getParam("/drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
  nh_private_.getParam("/drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
  nh_private_.getParam("/drrt/gain/kZero", params_.kZeroGain);
  nh_private_.getParam("/drrt/tree/kExtensionRange", params_.kExtensionRange);
  nh_private_.getParam("/drrt/tree/kMinExtensionRange",
                       params_.kMinextensionRange);
  nh_private_.getParam("/drrt/tree/kMaxExtensionAlongZ",
                       params_.kMaxExtensionAlongZ);
  nh_private_.getParam("/rrt/tree/kExactRoot", params_.kExactRoot);
  nh_private_.getParam("/drrt/tree/kCuttoffIterations",
                       params_.kCuttoffIterations);
  nh_private_.getParam("/drrt/tree/kGlobalExtraIterations",
                       params_.kGlobalExtraIterations);
  nh_private_.getParam("/drrt/tfFrame", params_.explorationFrame);
  nh_private_.getParam("/drrt/vertexSize", params_.kVertexSize);
  nh_private_.getParam("/drrt/keepTryingNum", params_.kKeepTryingNum);
  nh_private_.getParam("/lb/kMinXLocal", params_.kMinXLocalBound);
  nh_private_.getParam("/lb/kMinYLocal", params_.kMinYLocalBound);
  nh_private_.getParam("/lb/kMinZLocal", params_.kMinZLocalBound);
  nh_private_.getParam("/lb/kMaxXLocal", params_.kMaxXLocalBound);
  nh_private_.getParam("/lb/kMaxYLocal", params_.kMaxYLocalBound);
  nh_private_.getParam("/lb/kMaxZLocal", params_.kMaxZLocalBound);
  nh_private_.getParam("/gb/kMinXGlobal", params_.kMinXGlobalBound);
  nh_private_.getParam("/gb/kMinYGlobal", params_.kMinYGlobalBound);
  nh_private_.getParam("/gb/kMinZGlobal", params_.kMinZGlobalBound);
  nh_private_.getParam("/gb/kMaxXGlobal", params_.kMaxXGlobalBound);
  nh_private_.getParam("/gb/kMaxYGlobal", params_.kMaxYGlobalBound);
  nh_private_.getParam("/gb/kMaxZGlobal", params_.kMaxZGlobalBound);
  nh_private_.getParam("/cc/kObstacleHeightThre", params_.kObstacleHeightThre);
  nh_private_.getParam("/cc/kFlyingObstacleHeightThre",
                       params_.kFlyingObstacleHeightThre);
  nh_private_.getParam("/cc/kTerrainCheckDist", params_.kTerrainCheckDist);
  nh_private_.getParam("/cc/kTerrainCheckPointNum",
                       params_.kTerrainCheckPointNum);
  nh_private_.getParam("/cc/kTerrainVoxelSize", params_.kTerrainVoxelSize);
  nh_private_.getParam("/cc/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
  nh_private_.getParam("/cc/kTerrainVoxelHalfWidth",
                       params_.kTerrainVoxelHalfWidth);
  nh_private_.getParam("/planner/odomSubTopic", odomSubTopic);
  nh_private_.getParam("/planner/newTreePathPubTopic", newTreePathPubTopic);
  nh_private_.getParam("/planner/remainingTreePathPubTopic",
                       remainingTreePathPubTopic);
  nh_private_.getParam("/planner/boundaryPubTopic", boundaryPubTopic);
  nh_private_.getParam("/planner/globalSelectedFrontierPubTopic",
                       globalSelectedFrontierPubTopic);
  nh_private_.getParam("/planner/localSelectedFrontierPubTopic",
                       localSelectedFrontierPubTopic);
  nh_private_.getParam("/planner/plantimePubTopic", plantimePubTopic);
  nh_private_.getParam("/planner/nextGoalPubTopic", nextGoalPubTopic);
  nh_private_.getParam("/planner/pointInSensorRangePubTopic",
                       pointInSensorRangePubTopic);
  nh_private_.getParam("/planner/terrainNoGroundPubTopic",
                       terrainNoGroundPubTopic);
  nh_private_.getParam("/planner/shutDownTopic", shutDownTopic);
  nh_private_.getParam("/planner/plannerServiceName", plannerServiceName);
  nh_private_.getParam("/planner/cleanFrontierServiceName",
                       cleanFrontierServiceName);

  return true;
}

bool dsvplanner_ns::drrtPlanner::init() {
  if (!setParams()) {
    ROS_ERROR("Set parameters fail. Cannot start planning!");
  }

  odomSub_ = nh_.subscribe(odomSubTopic, 10,
                           &dsvplanner_ns::drrtPlanner::odomCallback, this);

  params_.newTreePathPub_ =
      nh_.advertise<visualization_msgs::Marker>(newTreePathPubTopic, 1000);
  params_.remainingTreePathPub_ = nh_.advertise<visualization_msgs::Marker>(
      remainingTreePathPubTopic, 1000);
  params_.boundaryPub_ =
      nh_.advertise<visualization_msgs::Marker>(boundaryPubTopic, 1000);
  params_.globalSelectedFrontierPub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      globalSelectedFrontierPubTopic, 1000);
  params_.localSelectedFrontierPub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      localSelectedFrontierPubTopic, 1000);
  params_.pointInSensorRangePub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(pointInSensorRangePubTopic, 1000);
  params_.terrainNoGroundPub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(terrainNoGroundPubTopic, 1000);
  params_.plantimePub_ =
      nh_.advertise<std_msgs::Float32>(plantimePubTopic, 1000);
  params_.nextGoalPub_ =
      nh_.advertise<geometry_msgs::PointStamped>(nextGoalPubTopic, 1000);
  params_.shutdownSignalPub =
      nh_.advertise<std_msgs::Bool>(shutDownTopic, 1000);

  plannerService_ = nh_.advertiseService(
      plannerServiceName, &dsvplanner_ns::drrtPlanner::plannerServiceCallback,
      this);
  cleanFrontierService_ = nh_.advertiseService(
      cleanFrontierServiceName,
      &dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback, this);

  drrt_->setParams(params_);
  return true;
}
