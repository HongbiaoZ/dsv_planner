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

#include "octomap_world/octomap_manager.h"

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace volumetric_mapping {
OctomapManager::OctomapManager(rclcpp::Node::SharedPtr& node_handle)
    : nh_(node_handle), world_frame_("map"),
      velodyne_cloud_topic_("/velodyne_cloud_topic"), robot_frame_("velodyne"),
      use_tf_transforms_(true), latch_topics_(true),
      timestamp_tolerance_ns_(10000000), Q_initialized_(false),
      Q_(Eigen::Matrix4d::Identity()), map_publish_frequency_(0.0) {
  setParametersFromROS();
  subscribe();
  advertiseServices();
  advertisePublishers();

  // After creating the manager, if the octomap_file parameter is set,
  // load the octomap at that path and publish it.
  std::string octomap_file;
  nh_->declare_parameter("octomap_file", octomap_file);
  if (nh_->get_parameter("octomap_file", octomap_file)) {
    if (loadOctomapFromFile(octomap_file)) {
      RCLCPP_INFO(nh_->get_logger(), "Successfully loaded octomap from path: %s", octomap_file.c_str());
      publishAll();
    } else {
      RCLCPP_ERROR(nh_->get_logger(), "Could not loaded octomap from path: %s", octomap_file.c_str());
    }
  }
}

void OctomapManager::setParametersFromROS() {
  OctomapParameters params;
  nh_->declare_parameter("octo/tfFrame", world_frame_);
  nh_->declare_parameter("octo/robotFrame", robot_frame_);
  nh_->declare_parameter("octo/resolution", params.resolution);
  nh_->declare_parameter("octo/probabilityHit", params.probabilityHit);
  nh_->declare_parameter("octo/probabilityMiss", params.probabilityMiss);
  nh_->declare_parameter("octo/thresholdMin", params.thresholdMin);
  nh_->declare_parameter("octo/thresholdMax", params.thresholdMax);
  nh_->declare_parameter("octo/thresholdOccupancy", params.thresholdOccupancy);
  nh_->declare_parameter("octo/filterSpeckles", params.filterSpeckles);
  nh_->declare_parameter("octo/maxFreeSpace", params.maxFreeSpace);
  nh_->declare_parameter("octo/minHeightFreeSpace", params.minHeightFreeSpace);
  nh_->declare_parameter("octo/sensorMaxRange", params.sensorMaxRange);
  nh_->declare_parameter("octo/visualizeMinZ", params.visualizeMinZ);
  nh_->declare_parameter("octo/visualizeMaxZ", params.visualizeMaxZ);
  nh_->declare_parameter("octo/mapPublishFrequency", map_publish_frequency_);
  nh_->declare_parameter("octo/treatUnknownAsOccupied",
                    params.treatUnknownAsOccupied);
  nh_->declare_parameter("octo/changeDetectionEnabled",
                    params.changeDetectionEnabled);
  nh_->declare_parameter("octo/velodyne_cloud_topic", velodyne_cloud_topic_);

  nh_->get_parameter("octo/tfFrame", world_frame_);
  nh_->get_parameter("octo/robotFrame", robot_frame_);
  nh_->get_parameter("octo/resolution", params.resolution);
  nh_->get_parameter("octo/probabilityHit", params.probabilityHit);
  nh_->get_parameter("octo/probabilityMiss", params.probabilityMiss);
  nh_->get_parameter("octo/thresholdMin", params.thresholdMin);
  nh_->get_parameter("octo/thresholdMax", params.thresholdMax);
  nh_->get_parameter("octo/thresholdOccupancy", params.thresholdOccupancy);
  nh_->get_parameter("octo/filterSpeckles", params.filterSpeckles);
  nh_->get_parameter("octo/maxFreeSpace", params.maxFreeSpace);
  nh_->get_parameter("octo/minHeightFreeSpace", params.minHeightFreeSpace);
  nh_->get_parameter("octo/sensorMaxRange", params.sensorMaxRange);
  nh_->get_parameter("octo/visualizeMinZ", params.visualizeMinZ);
  nh_->get_parameter("octo/visualizeMaxZ", params.visualizeMaxZ);
  nh_->get_parameter("octo/mapPublishFrequency", map_publish_frequency_);
  nh_->get_parameter("octo/treatUnknownAsOccupied",
                    params.treatUnknownAsOccupied);
  nh_->get_parameter("octo/changeDetectionEnabled",
                    params.changeDetectionEnabled);
  nh_->get_parameter("octo/velodyne_cloud_topic", velodyne_cloud_topic_);
  // Try to initialize Q matrix from parameters, if available.
  std::vector<double> Q_vec;

  // Set the parent class parameters.
  setOctomapParameters(params);
}

void OctomapManager::subscribe() {
  pointcloud_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(velodyne_cloud_topic_, 1, 
  std::bind(&OctomapManager::insertPointcloudWithTf, this, std::placeholders::_1));
  RCLCPP_INFO(nh_->get_logger(), "Successfully start sub1 node!!");
  octomap_sub_ = nh_->create_subscription<octomap_msgs::msg::Octomap>("input_octomap", 1, 
  std::bind(&OctomapManager::octomapCallback, this, std::placeholders::_1));
  RCLCPP_INFO(nh_->get_logger(), "Successfully start sub2 node!!");
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void OctomapManager::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  setOctomapFromMsg(*msg);
  publishAll();
  RCLCPP_INFO_ONCE(nh_->get_logger(), "Got octomap from message.");
}

void OctomapManager::advertiseServices() {
  
  reset_map_service_ = nh_->create_service<std_srvs::srv::Empty>("reset_map", std::bind(&OctomapManager::resetMapCallback, this, std::placeholders::_1, std::placeholders::_2));
  publish_all_service_ = nh_->create_service<std_srvs::srv::Empty>("publish_all", std::bind(&OctomapManager::publishAllCallback, this, std::placeholders::_1, std::placeholders::_2));
  get_map_service_ = nh_->create_service<octomap_msgs::srv::GetOctomap>("get_map", std::bind(&OctomapManager::getOctomapCallback, this, std::placeholders::_1, std::placeholders::_2));
  save_octree_service_ = nh_->create_service<volumetric_msgs::srv::SaveMap>("save_map", std::bind(&OctomapManager::saveOctomapCallback, this, std::placeholders::_1, std::placeholders::_2));
  load_octree_service_ = nh_->create_service<volumetric_msgs::srv::LoadMap>("load_map", std::bind(&OctomapManager::loadOctomapCallback, this, std::placeholders::_1, std::placeholders::_2));
  save_point_cloud_service_ = nh_->create_service<volumetric_msgs::srv::SaveMap>("save_point_cloud", std::bind(&OctomapManager::savePointCloudCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_box_occupancy_service_ = nh_->create_service<volumetric_msgs::srv::SetBoxOccupancy>("set_box_occupancy", std::bind(&OctomapManager::setBoxOccupancyCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_display_bounds_service_ = nh_->create_service<volumetric_msgs::srv::SetDisplayBounds>("set_display_bounds", std::bind(&OctomapManager::setDisplayBoundsCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void OctomapManager::advertisePublishers() {
  occupied_nodes_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("octomap_occupied", 1);
  free_nodes_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("octomap_free", 1);
  binary_map_pub_ = nh_->create_publisher<octomap_msgs::msg::Octomap>("octomap_binary", 1);
  full_map_pub_ = nh_->create_publisher<octomap_msgs::msg::Octomap>("octomap_full", 1);
  pcl_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("octomap_pcl", 1);
  nearest_obstacle_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("nearest_obstacle", 1);

  if (map_publish_frequency_ > 0.0) {
    map_publish_timer_ = nh_->create_wall_timer(1s, std::bind(&OctomapManager::publishAllEvent, this));
  }
}

void OctomapManager::publishAll() {
  if (latch_topics_ || occupied_nodes_pub_->get_subscription_count() > 0 ||
      free_nodes_pub_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray occupied_nodes, free_nodes;
    generateMarkerArray(world_frame_, &occupied_nodes, &free_nodes);
    occupied_nodes_pub_->publish(occupied_nodes);
    free_nodes_pub_->publish(free_nodes);
  }

  if (latch_topics_ || binary_map_pub_->get_subscription_count() > 0) {
    octomap_msgs::msg::Octomap binary_map;
    getOctomapBinaryMsg(&binary_map);
    binary_map.header.frame_id = world_frame_;
    binary_map_pub_->publish(binary_map);
  }

  if (latch_topics_ || full_map_pub_->get_subscription_count() > 0) {
    octomap_msgs::msg::Octomap full_map;
    getOctomapBinaryMsg(&full_map);
    full_map.header.frame_id = world_frame_;
    full_map_pub_->publish(full_map);
  }

  if (latch_topics_ || pcl_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    getOccupiedPointCloud(&point_cloud);
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(point_cloud, cloud);
    cloud.header.frame_id = world_frame_;
    pcl_pub_->publish(cloud);
  }

  if (use_tf_transforms_ && nearest_obstacle_pub_->get_subscription_count() > 0) {
    Transformation robot_to_world;
    if (lookupTransformTf(robot_frame_, world_frame_, nh_->now(),
                          &robot_to_world)) {
      Eigen::Vector3d robot_center = robot_to_world.getPosition();
      pcl::PointCloud<pcl::PointXYZ> point_cloud;
      getOccupiedPointcloudInBoundingBox(robot_center, robot_size_,
                                         &point_cloud);
      sensor_msgs::msg::PointCloud2 cloud;
      pcl::toROSMsg(point_cloud, cloud);
      cloud.header.frame_id = world_frame_;
      cloud.header.stamp = nh_->now();
      nearest_obstacle_pub_->publish(cloud);
    }
  }
}

void OctomapManager::publishAllEvent() { publishAll(); }

bool OctomapManager::resetMapCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
  resetMap();
  return true;
}

bool OctomapManager::publishAllCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
  publishAll();
  return true;
}

bool OctomapManager::getOctomapCallback(const octomap_msgs::srv::GetOctomap::Request::SharedPtr request,
                          octomap_msgs::srv::GetOctomap::Response::SharedPtr response) {
  return getOctomapFullMsg(&response->map);
}

bool OctomapManager::loadOctomapCallback(const volumetric_msgs::srv::LoadMap::Request::SharedPtr request,
                           volumetric_msgs::srv::LoadMap::Response::SharedPtr) {
  std::string extension =
      request->file_path.substr(request->file_path.find_last_of(".") + 1);
  if (extension == "bt") {
    return loadOctomapFromFile(request->file_path);
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (extension == "pcd") {
      pcl::io::loadPCDFile<pcl::PointXYZ>(request->file_path, *cloud);
    } else if (extension == "ply") {
      pcl::io::loadPLYFile<pcl::PointXYZ>(request->file_path, *cloud);
    } else {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "No known file extension (.bt, .pcd, .ply): " << request->file_path);
      return false;
    }
    octomap::KeySet free_cells, occupied_cells;
    for (size_t i = 0u; i < cloud->size(); ++i) {
      const octomap::point3d p_G_point((*cloud)[i].x, (*cloud)[i].y,
                                       (*cloud)[i].z);
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(p_G_point, key)) {
        occupied_cells.insert(key);
      }
    }
    updateOccupancy(&free_cells, &occupied_cells);
    return true;
  }
}

bool OctomapManager::saveOctomapCallback(volumetric_msgs::srv::SaveMap::Request::SharedPtr request,
                           volumetric_msgs::srv::SaveMap::Response::SharedPtr) {
  return writeOctomapToFile(request->file_path);
}

bool OctomapManager::savePointCloudCallback(volumetric_msgs::srv::SaveMap::Request::SharedPtr request,
                              volumetric_msgs::srv::SaveMap::Response::SharedPtr) {
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  getOccupiedPointCloud(&point_cloud);
  pcl::io::savePLYFileASCII(request->file_path, point_cloud);
  return true;
}

bool OctomapManager::setBoxOccupancyCallback(
      volumetric_msgs::srv::SetBoxOccupancy::Request::SharedPtr request,
      volumetric_msgs::srv::SetBoxOccupancy::Response::SharedPtr) {
  Eigen::Vector3d bounding_box_center;
  Eigen::Vector3d bounding_box_size;

  tf::vectorMsgToKindr(request->box_center, &bounding_box_center);
  tf::vectorMsgToKindr(request->box_size, &bounding_box_size);
  bool set_occupied = request->set_occupied;

  if (set_occupied) {
    setOccupied(bounding_box_center, bounding_box_size);
  } else {
    setFree(bounding_box_center, bounding_box_size);
  }
  publishAll();
  return true;
}

bool OctomapManager::setDisplayBoundsCallback(
      volumetric_msgs::srv::SetDisplayBounds::Request::SharedPtr request,
      volumetric_msgs::srv::SetDisplayBounds::Response::SharedPtr) {
  params_.visualizeMinZ = request->min_z;
  params_.visualizeMaxZ = request->max_z;
  publishAll();
  return true;
}

void OctomapManager::insertPointcloudWithTf(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud) {
  // Look up transform from sensor frame to world frame.
  Transformation sensor_to_world;
  if (lookupTransform(pointcloud->header.frame_id, world_frame_,
                      pointcloud->header.stamp, &sensor_to_world)) {
    insertPointcloud(sensor_to_world, pointcloud);
  }
}

bool OctomapManager::lookupTransform(const std::string &from_frame,
                                     const std::string &to_frame,
                                     const rclcpp::Time &timestamp,
                                     Transformation *transform) {
  if (use_tf_transforms_) {
    return lookupTransformTf(from_frame, to_frame, timestamp, transform);
  }
  else{
    return false;
  }
}

bool OctomapManager::lookupTransformTf(const std::string &from_frame,
                                       const std::string &to_frame,
                                       const rclcpp::Time &timestamp,
                                       Transformation *transform) {
  geometry_msgs::msg::TransformStamped tf2_geo_transform;

  rclcpp::Time time_to_lookup = timestamp;

  // If this transform isn't possible at the time, then try to just look up
  // the latest (this is to work with bag files and static transform publisher,
  // etc).
  if (!tf_buffer_->canTransform(to_frame, from_frame, tf2::TimePointZero)) {
    rclcpp::Duration timestamp_age = nh_->now() - time_to_lookup;
    if (timestamp_age < tf_buffer_->getCacheLength()) {
      time_to_lookup = nh_->now();
      // ROS_WARN("Using latest TF transform instead of timestamp match.");
    } else {
      // ROS_ERROR("Requested transform time older than cache limit.");
      return false;
    }
  }

  try {
    tf2_geo_transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "Error getting TF transform from sensor data: " << ex.what());
    return false;
  }

  tf2::Transform tf_transform;
  tf2::fromMsg(tf2_geo_transform.transform, tf_transform); 
  tf::transformTFToKindr(tf_transform, transform);
  return true;
}

} // namespace volumetric_mapping
