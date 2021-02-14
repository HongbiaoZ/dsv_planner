/**************************************************************************
dual_state_frontier.h
Header of the dual_state_frontier class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#ifndef DUAL_STATE_FRONTIER_H
#define DUAL_STATE_FRONTIER_H

#include "octomap_world/octomap_manager.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class DualStateFrontier {
public:
  typedef std::shared_ptr<DualStateFrontier> Ptr;
  typedef Eigen::Vector3d StateVec;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS subscribers
  // ros::Subscriber odom_sub_;
  ros::Subscriber graph_points_sub_;
  // ros::Subscriber terrain_point_cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>
      terrain_point_cloud_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
      nav_msgs::Odometry, sensor_msgs::PointCloud2>
      syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  // ROS publishers
  ros::Publisher unknown_points_pub_;
  ros::Publisher global_frontier_points_pub_;
  ros::Publisher local_frontier_points_pub_;
  ros::Publisher terrain_elev_cloud_pub_;

  // String constants
  std::string world_frame_id_;
  std::string sub_odom_topic_;
  std::string sub_graph_points_topic_;
  std::string sub_terrain_point_cloud_topic_;
  std::string pub_unknown_points_topic_;
  std::string pub_global_frontier_points_topic_;
  std::string pub_local_frontier_points_topic_;

  // Constants
  double kExecuteFrequency_;
  double kFrontierResolution;
  double kFrontierFilterSize;
  double kSearchRadius;
  double kSensorVerticalView;
  double kSensorHorizontalView;
  double kVehicleHeight;
  double kGlobalMaxX;
  double kGlobalMaxY;
  double kGlobalMaxZ;
  double kGlobalMinX;
  double kGlobalMinY;
  double kGlobalMinZ;

  StateVec robot_bounding;
  StateVec search_bounding;

  double kFlyingObstacleHeightThre;
  double kObstacleHeightThre;
  double kTerrainCheckDist;
  double kTerrainCheckPointNum;
  double kTerrainVoxelSize;
  int kTerrainVoxelHalfWidth;
  int kTerrainVoxelWidth;

  // Variables

  ros::Timer executeTimer_;
  std::vector<double> terrain_voxel_elev_;
  std::vector<int> terrain_voxel_points_num_;
  std::vector<double> terrain_voxel_min_elev_;
  std::vector<double> terrain_voxel_max_elev_;
  StateVec robot_position_;
  bool planner_status_; // 0 means exploration and 1 means relocation
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ds =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_crop_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr unknown_points_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_frontier_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_frontier_pcl_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_pcl_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cleanedFrontier_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr graphPoints_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_elev_cloud_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_ =
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(
          new pcl::KdTreeFLANN<pcl::PointXYZ>());

  volumetric_mapping::OctomapManager *manager_;

  // General Functions
  void getUnknowPointcloudInBoundingBox(const StateVec &center,
                                        const StateVec &bounding_box_size);
  void getFrontiers();
  void publishFrontiers();
  void updateToCleanFrontier(pcl::PointXYZ point);
  void gloabalFrontierUpdate();
  void localFrontierUpdate(StateVec &center);
  void cleanAllUselessFrontiers();
  void setPlannerStatus(bool status);
  bool frontierDetect(octomap::point3d point) const;
  bool collisionCheckByTerrain(StateVec origin_point, StateVec goal_point);
  bool inSensorRangeofGraphPoints(StateVec point);
  bool FrontierInBoundry(octomap::point3d point) const;
  bool isCleanedFrontier(pcl::PointXYZ point);
  double getZvalue(double x_position, double y_position);
  pcl::PointCloud<pcl::PointXYZI> getTerrainPoints();
  std::vector<double> getTerrainVoxelElev();

  // Callback Functions
  void terrainCloudAndOdomCallback(
      const nav_msgs::Odometry::ConstPtr &odom_msg,
      const sensor_msgs::PointCloud2::ConstPtr &terrain_msg);
  void graphPointsCallback(const sensor_msgs::PointCloud2 &graph_msg);

public:
  DualStateFrontier(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private,
                    volumetric_mapping::OctomapManager *manager);
  bool readParameters();
  bool initialize();
  void execute(const ros::TimerEvent &e);
  ~DualStateFrontier();
};
#endif // DUAL_STATE_FRONTIER_H
