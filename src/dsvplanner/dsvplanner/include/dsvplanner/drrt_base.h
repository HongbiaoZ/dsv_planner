/**************************************************************************
drrt_base.h
Header of the based class for drrt

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/
#ifndef DRRT_BASE_H_
#define DRRT_BASE_H_

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

using namespace Eigen;
namespace dsvplanner_ns
{
struct Params
{
  ros::Publisher newTreePathPub_;
  ros::Publisher remainingTreePathPub_;
  ros::Publisher boundaryPub_;
  ros::Publisher globalSelectedFrontierPub_;
  ros::Publisher localSelectedFrontierPub_;
  ros::Publisher nextGoalPub_;
  ros::Publisher plantimePub_;
  ros::Publisher randomSampledPointsPub_;
  ros::Publisher shutdownSignalPub;

  double sensorPitch;
  double sensorHorizontalView;
  double sensorVerticalView;
  double kVehicleHeight;
  Eigen::Vector3d boundingBox;
  Eigen::Vector3d localBoundary;
  Eigen::Vector3d globalBoundary;

  double kGainFree;
  double kGainOccupied;
  double kGainUnknown;
  double kGainRange;
  double kGainRangeZMinus;
  double kGainRangeZPlus;
  double kZeroGain;

  double kExtensionRange;
  double kMinextensionRange;
  double kMaxExtensionAlongZ;
  bool kExactRoot;
  int kMinEffectiveGain;
  int kGlobalExtraIterations;
  int kCuttoffIterations;
  int kVertexSize;
  int kKeepTryingNum;
  int kLoopCountThres;

  double kMinXLocalBound;
  double kMinYLocalBound;
  double kMinZLocalBound;
  double kMaxXLocalBound;
  double kMaxYLocalBound;
  double kMaxZLocalBound;
  double kMinXGlobalBound;
  double kMinYGlobalBound;
  double kMinZGlobalBound;
  double kMaxXGlobalBound;
  double kMaxYGlobalBound;
  double kMaxZGlobalBound;

  double kTerrainVoxelSize;
  int kTerrainVoxelWidth;
  int kTerrainVoxelHalfWidth;

  double kRemainingNodeScaleSize;
  double kRemainingBranchScaleSize;
  double kNewNodeScaleSize;
  double kNewBranchScaleSize;

  std::string explorationFrame;
};

class Node
{
public:
  Node(){};
  ~Node(){};
  Vector3d state_;
  Node* parent_;
  std::vector<Node*> children_;
  double gain_;
  double distance_;
};
}

#endif  // DRRT_BASE_H_
