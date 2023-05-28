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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define SQ(x) ((x) * (x))
#define SQRT2 0.70711

using namespace Eigen;
namespace dsvplanner_ns
{
struct Params
{
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr newTreePathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr remainingTreePathPub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundaryPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalSelectedFrontierPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localSelectedFrontierPub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr nextGoalPub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr plantimePub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr randomSampledPointsPub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shutdownSignalPub;

  double sensorPitch;
  double sensorHorizontalView;
  double sensorVerticalView;
  double kVehicleHeight;
  Vector3d boundingBox;
  Vector3d localBoundary;
  Vector3d globalBoundary;

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
