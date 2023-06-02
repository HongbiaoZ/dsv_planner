/*
drrtp.cpp
Implementation of drrt_planner class

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>

#include <dsvplanner/drrtp.h>

using namespace Eigen;

dsvplanner_ns::drrtPlanner::drrtPlanner(rclcpp::Node::SharedPtr& node_handle)
  : nh_(node_handle)
{
  if (!setParams())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Set parameters fail. Cannot start planning!");
  }

  manager_ = new volumetric_mapping::OctomapManager(nh_);
  RCLCPP_INFO(nh_->get_logger(), "Successfully launched octomap node");
  grid_ = new OccupancyGrid(nh_);
  RCLCPP_INFO(nh_->get_logger(), "Successfully launched grid node");
  dual_state_graph_ = new DualStateGraph(nh_, manager_, grid_);
  RCLCPP_INFO(nh_->get_logger(), "Successfully launched graph node");
  dual_state_frontier_ = new DualStateFrontier(nh_, manager_, grid_);
  RCLCPP_INFO(nh_->get_logger(), "Successfully launched frontier node");
  drrt_ = new Drrt(nh_, manager_, dual_state_graph_, dual_state_frontier_, grid_);
  RCLCPP_INFO(nh_->get_logger(), "Successfully launched drrt node");

  init();
  drrt_->setParams(params_);
  drrt_->init();

  RCLCPP_INFO(nh_->get_logger(), "Successfully launched DSVP node");
}

dsvplanner_ns::drrtPlanner::~drrtPlanner()
{
  if (manager_)
  {
    delete manager_;
  }
  if (dual_state_graph_)
  {
    delete dual_state_graph_;
  }
  if (dual_state_frontier_)
  {
    delete dual_state_frontier_;
  }
  if (grid_)
  {
    delete grid_;
  }
  if (drrt_)
  {
    delete drrt_;
  }
}

void dsvplanner_ns::drrtPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr pose)
{
  drrt_->setRootWithOdom(*pose);
  // Planner is now ready to plan.
  drrt_->plannerReady_ = true;
}

void dsvplanner_ns::drrtPlanner::boundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr boundary)
{
  drrt_->setBoundary(*boundary);
  dual_state_frontier_->setBoundary(*boundary);
}

void dsvplanner_ns::drrtPlanner::plannerServiceCallback(const dsvplanner::srv::Dsvplanner::Request::SharedPtr req, 
                                                              dsvplanner::srv::Dsvplanner::Response::SharedPtr res)
{
  plan_start_ = std::chrono::steady_clock::now();
  // drrt_->gotoxy(0, 10);  // Go to the specific line on the screen
  // Check if the planner is ready.
  if (!drrt_->plannerReady_)
  {
    std::cout << "No odometry. Planner is not ready!" << std::endl;
    return;
  }
  if (manager_ == NULL)
  {
    std::cout << "No octomap. Planner is not ready!" << std::endl;
    return;
  }
  if (manager_->getMapSize().norm() <= 0.0)
  {
    std::cout << "Octomap is empty. Planner is not set up!" << std::endl;
    return;
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
  while (rclcpp::ok() && drrt_->remainingFrontier_ && drrt_->getNodeCounter() < params_.kCuttoffIterations &&
         !(drrt_->normal_local_iteration_ && (drrt_->getNodeCounter() >= params_.kVertexSize && drrt_->gainFound())))
  {
    if (loopCount > drrt_->loopCount_ * (drrt_->getNodeCounter() + 1))
    {
      break;
    }
    drrt_->plannerIterate();
    loopCount++;
  }

  // Publish rrt
  drrt_->publishNode();
  std::cout << "     New node number is " << drrt_->getNodeCounter() << "\n"
            << "     Current local RRT size is " << dual_state_graph_->getLocalVertexSize() << "\n"
            << "     Current global graph size is " << dual_state_graph_->getGlobalVertexSize() << std::endl;
  RRT_generate_over_ = std::chrono::steady_clock::now();
  time_span = RRT_generate_over_ - plan_start_;
  double rrtGenerateTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  // Reset planner state
  drrt_->global_plan_pre_ = drrt_->global_plan_;
  drrt_->global_plan_ = false;
  drrt_->local_plan_ = false;

  // update planner state of last iteration
  dual_state_frontier_->setPlannerStatus(drrt_->global_plan_pre_);

  // Update planner state of next iteration
  geometry_msgs::msg::Point robot_position;
  robot_position.x = drrt_->root_[0];
  robot_position.y = drrt_->root_[1];
  robot_position.z = drrt_->root_[2];
  if (!drrt_->nextNodeFound_ && drrt_->global_plan_pre_ && drrt_->gainFound() <= 0)
  {
    drrt_->return_home_ = true;
    geometry_msgs::msg::Point home_position;
    home_position.x = 0;
    home_position.y = 0;
    home_position.z = 0;
    res->goal = home_position;
    res->mode = 2;  // mode 2 means returning home

    dual_state_frontier_->cleanAllUselessFrontiers();
    return;
  }
  else if (!drrt_->nextNodeFound_ && !drrt_->global_plan_pre_ && dual_state_graph_->getGain(robot_position) <= 0)
  {
    drrt_->global_plan_ = true;
    std::cout << "     No Remaining local frontiers  "
              << "\n"
              << "     Switch to relocation stage "
              << "\n"
              << "     Total plan lasted " << 0 << std::endl;
    return;
  }
  else
  {
    drrt_->local_plan_ = true;
  }
  gain_computation_over_ = std::chrono::steady_clock::now();
  time_span = gain_computation_over_ - RRT_generate_over_;
  double getGainTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;

  // Extract next goal.
  geometry_msgs::msg::Point next_goal_position;
  if (drrt_->nextNodeFound_)
  {
    dual_state_graph_->best_vertex_id_ = drrt_->NextBestNodeIdx_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestGlobalVertexPosition();
  }
  else if (drrt_->global_plan_pre_ == true && drrt_->gainFound())
  {
    dual_state_graph_->best_vertex_id_ = drrt_->bestNodeId_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  else
  {
    dual_state_graph_->updateGlobalGraph();
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  dual_state_graph_->setCurrentPlannerStatus(drrt_->global_plan_pre_);
  res->goal = next_goal_position;
  res->mode = 1;  // mode 1 means exploration

  geometry_msgs::msg::PointStamped next_goal_point;
  next_goal_point.header.frame_id = "map";
  next_goal_point.point = next_goal_position;
  params_.nextGoalPub_->publish(next_goal_point);

  plan_over_ = std::chrono::steady_clock::now();
  time_span = plan_over_ - plan_start_;
  double plantime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  std::cout << "     RRT generation lasted  " << rrtGenerateTime << "\n"
            << "     Computiong gain lasted " << getGainTime << "\n"
            << "     Total plan lasted " << plantime << std::endl;
}

void dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback(const dsvplanner::srv::CleanFrontier::Request::SharedPtr req,
                                                                    dsvplanner::srv::CleanFrontier::Response::SharedPtr res)
{
  if (drrt_->nextNodeFound_)
  {
    dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
    dual_state_frontier_->gloabalFrontierUpdate();
  }
  else
  {
    dual_state_graph_->clearLocalGraph();
  }
  res->success = true;

}

void dsvplanner_ns::drrtPlanner::cleanLastSelectedGlobalFrontier()
{
  // only when last plan is global plan, this function will be executed to clear
  // last selected global
  // frontier.
  if (drrt_->nextNodeFound_)
  {
    dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
    dual_state_frontier_->gloabalFrontierUpdate();
  }
}

bool dsvplanner_ns::drrtPlanner::setParams()
{
  nh_->declare_parameter("drrt/gain/kFree", params_.kGainFree);
  nh_->declare_parameter("drrt/gain/kOccupied", params_.kGainOccupied);
  nh_->declare_parameter("drrt/gain/kUnknown", params_.kGainUnknown);
  nh_->declare_parameter("drrt/gain/kMinEffectiveGain", params_.kMinEffectiveGain);
  nh_->declare_parameter("drrt/gain/kRange", params_.kGainRange);
  nh_->declare_parameter("drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
  nh_->declare_parameter("drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
  nh_->declare_parameter("drrt/gain/kZero", params_.kZeroGain);
  nh_->declare_parameter("drrt/tree/kExtensionRange", params_.kExtensionRange);
  nh_->declare_parameter("drrt/tree/kMinExtensionRange", params_.kMinextensionRange);
  nh_->declare_parameter("drrt/tree/kMaxExtensionAlongZ", params_.kMaxExtensionAlongZ);
  nh_->declare_parameter("drrt/tree/kExactRoot", params_.kExactRoot);
  nh_->declare_parameter("drrt/tree/kCuttoffIterations", params_.kCuttoffIterations);
  nh_->declare_parameter("drrt/tree/kGlobalExtraIterations", params_.kGlobalExtraIterations);
  nh_->declare_parameter("drrt/tree/kRemainingNodeScaleSize", params_.kRemainingNodeScaleSize);
  nh_->declare_parameter("drrt/tree/kRemainingBranchScaleSize", params_.kRemainingBranchScaleSize);
  nh_->declare_parameter("drrt/tree/kNewNodeScaleSize", params_.kNewNodeScaleSize);
  nh_->declare_parameter("drrt/tree/kNewBranchScaleSize", params_.kNewBranchScaleSize);
  nh_->declare_parameter("drrt/tfFrame", params_.explorationFrame);
  nh_->declare_parameter("drrt/vertexSize", params_.kVertexSize);
  nh_->declare_parameter("drrt/keepTryingNum", params_.kKeepTryingNum);
  nh_->declare_parameter("drrt/kLoopCountThres", params_.kLoopCountThres);

  nh_->declare_parameter("planner/odomSubTopic", odomSubTopic);
  nh_->declare_parameter("planner/boundarySubTopic", boundarySubTopic);
  nh_->declare_parameter("planner/newTreePathPubTopic", newTreePathPubTopic);
  nh_->declare_parameter("planner/remainingTreePathPubTopic", remainingTreePathPubTopic);
  nh_->declare_parameter("planner/boundaryPubTopic", boundaryPubTopic);
  nh_->declare_parameter("planner/globalSelectedFrontierPubTopic", globalSelectedFrontierPubTopic);
  nh_->declare_parameter("planner/localSelectedFrontierPubTopic", localSelectedFrontierPubTopic);
  nh_->declare_parameter("planner/plantimePubTopic", plantimePubTopic);
  nh_->declare_parameter("planner/nextGoalPubTopic", nextGoalPubTopic);
  nh_->declare_parameter("planner/randomSampledPointsPubTopic", randomSampledPointsPubTopic);
  nh_->declare_parameter("planner/shutDownTopic", shutDownTopic);
  nh_->declare_parameter("planner/plannerServiceName", plannerServiceName);
  nh_->declare_parameter("planner/cleanFrontierServiceName", cleanFrontierServiceName);

  nh_->declare_parameter("lb/kMinXLocal", params_.kMinXLocalBound);
  nh_->declare_parameter("lb/kMinYLocal", params_.kMinYLocalBound);
  nh_->declare_parameter("lb/kMinZLocal", params_.kMinZLocalBound);
  nh_->declare_parameter("lb/kMaxXLocal", params_.kMaxXLocalBound);
  nh_->declare_parameter("lb/kMaxYLocal", params_.kMaxYLocalBound);
  nh_->declare_parameter("lb/kMaxZLocal", params_.kMaxZLocalBound);

  nh_->declare_parameter("gb/kMinXGlobal", params_.kMinXGlobalBound);
  nh_->declare_parameter("gb/kMinYGlobal", params_.kMinYGlobalBound);
  nh_->declare_parameter("gb/kMinZGlobal", params_.kMinZGlobalBound);
  nh_->declare_parameter("gb/kMaxXGlobal", params_.kMaxXGlobalBound);
  nh_->declare_parameter("gb/kMaxYGlobal", params_.kMaxYGlobalBound);
  nh_->declare_parameter("gb/kMaxZGlobal", params_.kMaxZGlobalBound);

  nh_->declare_parameter("rm/kSensorPitch", params_.sensorPitch);
  nh_->declare_parameter("rm/kSensorHorizontal", params_.sensorHorizontalView);
  nh_->declare_parameter("rm/kSensorVertical", params_.sensorVerticalView);
  nh_->declare_parameter("rm/kVehicleHeight", params_.kVehicleHeight);
  nh_->declare_parameter("rm/kBoundX", params_.boundingBox[0]);
  nh_->declare_parameter("rm/kBoundY", params_.boundingBox[1]);
  nh_->declare_parameter("rm/kBoundZ", params_.boundingBox[2]);

  nh_->declare_parameter("elevation/kTerrainVoxelSize", params_.kTerrainVoxelSize);
  nh_->declare_parameter("elevation/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
  nh_->declare_parameter("elevation/kTerrainVoxelHalfWidth", params_.kTerrainVoxelHalfWidth);

  nh_->get_parameter("rm/kSensorPitch", params_.sensorPitch);
  nh_->get_parameter("rm/kSensorHorizontal", params_.sensorHorizontalView);
  nh_->get_parameter("rm/kSensorVertical", params_.sensorVerticalView);
  nh_->get_parameter("rm/kVehicleHeight", params_.kVehicleHeight);
  nh_->get_parameter("rm/kBoundX", params_.boundingBox[0]);
  nh_->get_parameter("rm/kBoundY", params_.boundingBox[1]);
  nh_->get_parameter("rm/kBoundZ", params_.boundingBox[2]);
  nh_->get_parameter("drrt/gain/kFree", params_.kGainFree);
  nh_->get_parameter("drrt/gain/kOccupied", params_.kGainOccupied);
  nh_->get_parameter("drrt/gain/kUnknown", params_.kGainUnknown);
  nh_->get_parameter("drrt/gain/kMinEffectiveGain", params_.kMinEffectiveGain);
  nh_->get_parameter("drrt/gain/kRange", params_.kGainRange);
  nh_->get_parameter("drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
  nh_->get_parameter("drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
  nh_->get_parameter("drrt/gain/kZero", params_.kZeroGain);
  nh_->get_parameter("drrt/tree/kExtensionRange", params_.kExtensionRange);
  nh_->get_parameter("drrt/tree/kMinExtensionRange", params_.kMinextensionRange);
  nh_->get_parameter("drrt/tree/kMaxExtensionAlongZ", params_.kMaxExtensionAlongZ);
  nh_->get_parameter("/rrt/tree/kExactRoot", params_.kExactRoot);
  nh_->get_parameter("drrt/tree/kCuttoffIterations", params_.kCuttoffIterations);
  nh_->get_parameter("drrt/tree/kGlobalExtraIterations", params_.kGlobalExtraIterations);
  nh_->get_parameter("drrt/tree/kRemainingNodeScaleSize", params_.kRemainingNodeScaleSize);
  nh_->get_parameter("drrt/tree/kRemainingBranchScaleSize", params_.kRemainingBranchScaleSize);
  nh_->get_parameter("drrt/tree/kNewNodeScaleSize", params_.kNewNodeScaleSize);
  nh_->get_parameter("drrt/tree/kNewBranchScaleSize", params_.kNewBranchScaleSize);
  nh_->get_parameter("drrt/tfFrame", params_.explorationFrame);
  nh_->get_parameter("drrt/vertexSize", params_.kVertexSize);
  nh_->get_parameter("drrt/keepTryingNum", params_.kKeepTryingNum);
  nh_->get_parameter("drrt/kLoopCountThres", params_.kLoopCountThres);
  nh_->get_parameter("lb/kMinXLocal", params_.kMinXLocalBound);
  nh_->get_parameter("lb/kMinYLocal", params_.kMinYLocalBound);
  nh_->get_parameter("lb/kMinZLocal", params_.kMinZLocalBound);
  nh_->get_parameter("lb/kMaxXLocal", params_.kMaxXLocalBound);
  nh_->get_parameter("lb/kMaxYLocal", params_.kMaxYLocalBound);
  nh_->get_parameter("lb/kMaxZLocal", params_.kMaxZLocalBound);
  nh_->get_parameter("gb/kMinXGlobal", params_.kMinXGlobalBound);
  nh_->get_parameter("gb/kMinYGlobal", params_.kMinYGlobalBound);
  nh_->get_parameter("gb/kMinZGlobal", params_.kMinZGlobalBound);
  nh_->get_parameter("gb/kMaxXGlobal", params_.kMaxXGlobalBound);
  nh_->get_parameter("gb/kMaxYGlobal", params_.kMaxYGlobalBound);
  nh_->get_parameter("gb/kMaxZGlobal", params_.kMaxZGlobalBound);
  nh_->get_parameter("elevation/kTerrainVoxelSize", params_.kTerrainVoxelSize);
  nh_->get_parameter("elevation/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
  nh_->get_parameter("elevation/kTerrainVoxelHalfWidth", params_.kTerrainVoxelHalfWidth);
  nh_->get_parameter("planner/odomSubTopic", odomSubTopic);
  nh_->get_parameter("planner/boundarySubTopic", boundarySubTopic);
  nh_->get_parameter("planner/newTreePathPubTopic", newTreePathPubTopic);
  nh_->get_parameter("planner/remainingTreePathPubTopic", remainingTreePathPubTopic);
  nh_->get_parameter("planner/boundaryPubTopic", boundaryPubTopic);
  nh_->get_parameter("planner/globalSelectedFrontierPubTopic", globalSelectedFrontierPubTopic);
  nh_->get_parameter("planner/localSelectedFrontierPubTopic", localSelectedFrontierPubTopic);
  nh_->get_parameter("planner/plantimePubTopic", plantimePubTopic);
  nh_->get_parameter("planner/nextGoalPubTopic", nextGoalPubTopic);
  nh_->get_parameter("planner/randomSampledPointsPubTopic", randomSampledPointsPubTopic);
  nh_->get_parameter("planner/shutDownTopic", shutDownTopic);
  nh_->get_parameter("planner/plannerServiceName", plannerServiceName);
  nh_->get_parameter("planner/cleanFrontierServiceName", cleanFrontierServiceName);

  return true;
}

bool dsvplanner_ns::drrtPlanner::init()
{

  odomSub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(odomSubTopic, 1, 
  std::bind(&dsvplanner_ns::drrtPlanner::odomCallback, this, std::placeholders::_1));

  boundarySub_ = nh_->create_subscription<geometry_msgs::msg::PolygonStamped>(boundarySubTopic, 1, 
  std::bind(&dsvplanner_ns::drrtPlanner::boundaryCallback, this, std::placeholders::_1));

  params_.newTreePathPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>(newTreePathPubTopic, 1000);
  params_.remainingTreePathPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>(remainingTreePathPubTopic, 1000);
  params_.boundaryPub_ = nh_->create_publisher<visualization_msgs::msg::Marker>(boundaryPubTopic, 1000);
  params_.globalSelectedFrontierPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(globalSelectedFrontierPubTopic, 1000);
  params_.localSelectedFrontierPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(localSelectedFrontierPubTopic, 1000);
  params_.randomSampledPointsPub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(randomSampledPointsPubTopic, 1000);
  params_.plantimePub_ = nh_->create_publisher<std_msgs::msg::Float32>(plantimePubTopic, 1000);
  params_.nextGoalPub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>(nextGoalPubTopic, 1000);
  params_.shutdownSignalPub = nh_->create_publisher<std_msgs::msg::Bool>(shutDownTopic, 1);

  plannerService_ = nh_->create_service<dsvplanner::srv::Dsvplanner>(plannerServiceName, std::bind(&dsvplanner_ns::drrtPlanner::plannerServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  cleanFrontierService_ = nh_->create_service<dsvplanner::srv::CleanFrontier>(cleanFrontierServiceName, std::bind(&dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  return true;
}
