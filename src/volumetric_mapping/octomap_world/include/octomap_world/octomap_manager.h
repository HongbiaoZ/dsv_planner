/*
Copyright (c) 2015, Helen Oleynikova, ETH Zurich, Switzerland
You can contact the author at <helen dot oleynikova at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
#define OCTOMAP_WORLD_OCTOMAP_MANAGER_H_

#include <chrono>

#include <octomap_msgs/srv/get_octomap.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2/exceptions.h"
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <volumetric_msgs/srv/load_map.hpp>
#include <volumetric_msgs/srv/save_map.hpp>
#include <volumetric_msgs/srv/set_box_occupancy.hpp>
#include <volumetric_msgs/srv/set_display_bounds.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "octomap_world/octomap_world.h"


using namespace std::chrono_literals;

namespace volumetric_mapping {

// An inherited class from OctomapWorld, which also handles the connection to
// ROS via publishers, subscribers, service calls, etc.
class OctomapManager : public OctomapWorld {
 public:
  typedef std::shared_ptr<OctomapManager> Ptr;

  // By default, loads octomap parameters from the ROS parameter server.
  OctomapManager(rclcpp::Node::SharedPtr& node_handle);

  void publishAll();
  void publishAllEvent();

  // Data insertion callbacks with TF frame resolution through the listener.
  void insertPointcloudWithTf(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud);

  // Input Octomap callback.
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // Service callbacks.
  bool resetMapCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool publishAllCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool getOctomapCallback(const octomap_msgs::srv::GetOctomap::Request::SharedPtr request,
                          octomap_msgs::srv::GetOctomap::Response::SharedPtr);

  bool loadOctomapCallback(const volumetric_msgs::srv::LoadMap::Request::SharedPtr request,
                           volumetric_msgs::srv::LoadMap::Response::SharedPtr);
  bool saveOctomapCallback(volumetric_msgs::srv::SaveMap::Request::SharedPtr request,
                           volumetric_msgs::srv::SaveMap::Response::SharedPtr);
  bool savePointCloudCallback(volumetric_msgs::srv::SaveMap::Request::SharedPtr request,
                              volumetric_msgs::srv::SaveMap::Response::SharedPtr);

  bool setBoxOccupancyCallback(
      volumetric_msgs::srv::SetBoxOccupancy::Request::SharedPtr request,
      volumetric_msgs::srv::SetBoxOccupancy::Response::SharedPtr);
  bool setDisplayBoundsCallback(
      volumetric_msgs::srv::SetDisplayBounds::Request::SharedPtr request,
      volumetric_msgs::srv::SetDisplayBounds::Response::SharedPtr);

 private:
  // Sets up subscriptions based on ROS node parameters.
  void setParametersFromROS();
  void subscribe();
  void advertiseServices();
  void advertisePublishers();

  bool lookupTransform(const std::string& from_frame,
                       const std::string& to_frame, const rclcpp::Time& timestamp,
                       Transformation* transform);
  bool lookupTransformTf(const std::string& from_frame,
                         const std::string& to_frame,
                         const rclcpp::Time& timestamp, Transformation* transform);

  rclcpp::Node::SharedPtr nh_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Global/map coordinate frame. Will always look up TF transforms to this
  // frame.
  std::string world_frame_;
  std::string robot_frame_;
  // Whether to use TF transform resolution (true) or fixed transforms from
  // parameters and transform topics (false).
  bool use_tf_transforms_;
  int64_t timestamp_tolerance_ns_;

  //Topic name of velodyne points
  std::string velodyne_cloud_topic_;

  bool latch_topics_;
  // Subscriptions for input sensor data.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  // Publish full state of octomap.
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_map_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr full_map_pub_;

  // Publish voxel centroids as pcl.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nearest_obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

  // Publish markers for visualization.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_nodes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr free_nodes_pub_;


  // Services!
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_map_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr publish_all_service_;
  rclcpp::Service<octomap_msgs::srv::GetOctomap>::SharedPtr get_map_service_;
  rclcpp::Service<volumetric_msgs::srv::SaveMap>::SharedPtr save_octree_service_;
  rclcpp::Service<volumetric_msgs::srv::LoadMap>::SharedPtr load_octree_service_;
  rclcpp::Service<volumetric_msgs::srv::SaveMap>::SharedPtr save_point_cloud_service_;
  rclcpp::Service<volumetric_msgs::srv::SetBoxOccupancy>::SharedPtr set_box_occupancy_service_;
  rclcpp::Service<volumetric_msgs::srv::SetDisplayBounds>::SharedPtr set_display_bounds_service_;

  // Only calculate Q matrix for disparity once.
  bool Q_initialized_;
  Eigen::Matrix4d Q_;
  double map_publish_frequency_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  // Transform queue, used only when use_tf_transforms is false.
  std::deque<geometry_msgs::msg::TransformStamped> transform_queue_;

};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_MANAGER_H_
