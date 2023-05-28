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

#ifndef OCTOMAP_WORLD_OCTOMAP_WORLD_H_
#define OCTOMAP_WORLD_OCTOMAP_WORLD_H_

#include <string>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <volumetric_map_base/world_base.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace volumetric_mapping {

struct OctomapParameters {
  OctomapParameters()
      : resolution(0.15),
        probabilityHit(0.65),
        probabilityMiss(0.4),
        thresholdMin(0.12),
        thresholdMax(0.97),
        thresholdOccupancy(0.7),
        filterSpeckles(true),
        maxFreeSpace(0.0),
        minHeightFreeSpace(0.0),
        sensorMaxRange(5.0),
        visualizeMinZ(-std::numeric_limits<double>::max()),
        visualizeMaxZ(std::numeric_limits<double>::max()),
        treatUnknownAsOccupied(true),
        changeDetectionEnabled(false){
    // Set reasonable defaults here...
  }

  // Resolution for the Octree. It is not possible to change this without
  // creating a new Octree.
  double resolution;
  // Hit probabilities for pointcloud data.
  double probabilityHit;
  double probabilityMiss;
  // Clamping thresholds for pruning: above and below these thresholds, all
  // values are treated the same.
  double thresholdMin;
  double thresholdMax;
  // Threshold considered for a cell to be occupied.
  double thresholdOccupancy;

  // Filter neighbor-less nodes as 'speckles'.
  bool filterSpeckles;

  // Maximum range to allow a free space update.
  double maxFreeSpace;

  // Minimum height below sensor to allow a free space update.
  double minHeightFreeSpace;

  // Maximum range to allow a sensor measurement. Negative values to not
  // filter.
  double sensorMaxRange;

  // Minimum and maximum z to visualize. Only used for marker, not full
  // octomap, visualization.
  double visualizeMinZ;
  double visualizeMaxZ;

  // Collision checking.
  bool treatUnknownAsOccupied;

  // Whether to track changes -- must be set to true to use getChangedPoints().
  bool changeDetectionEnabled;

};

// A wrapper around octomap that allows insertion from various ROS message
// data sources, given their transforms from sensor frame to world frame.
// Does not need to run within a ROS node, does not do any TF look-ups, and
// does not publish/subscribe to anything (though provides serialization
// and deserialization functions to and from ROS messages).
class OctomapWorld : public WorldBase {
  typedef std::shared_ptr<OctomapWorld> Ptr;
  typedef Eigen::Vector3d StateVec;

 public:
  // Default constructor - creates a valid octree using parameter defaults.
  OctomapWorld();

  // Creates an octomap with the correct parameters.
  OctomapWorld(const OctomapParameters& params);
  virtual ~OctomapWorld() {}

  // General map management.
  void resetMap();
  void prune();
  // Creates an octomap if one is not yet created or if the resolution of the
  // current varies from the parameters requested.
  void setOctomapParameters(const OctomapParameters& params);

  // Virtual functions for manually manipulating map probabilities.
  virtual void setFree(const StateVec& position,
                       const StateVec& bounding_box_size);
  virtual void setOccupied(const StateVec& position,
                           const StateVec& bounding_box_size);

  // Virtual functions for outputting map status.
  virtual CellStatus getCellStatusBoundingBox(
      const StateVec& point,
      const StateVec& bounding_box_size) const;
  virtual CellStatus getCellStatusPoint(const StateVec& point) const;
  virtual CellStatus getCellProbabilityPoint(const StateVec& point,
                                             double* probability) const;
  virtual CellStatus getLineStatus(const StateVec& start,
                                   const StateVec& end) const;
  virtual CellStatus getVisibility(const StateVec& view_point,
                                   const StateVec& voxel_to_test,
                                   bool stop_at_unknown_cell) const;
  virtual CellStatus getLineStatusBoundingBox(
      const StateVec& start, const StateVec& end,
      const StateVec& bounding_box_size) const;
  virtual void getOccupiedPointCloud(
      pcl::PointCloud<pcl::PointXYZ>* output_cloud) const;
  virtual void getOccupiedPointcloudInBoundingBox(
      const StateVec& center, const StateVec& bounding_box_size,
      pcl::PointCloud<pcl::PointXYZ>* output_cloud) const;

  // Structure: vector of pairs, key is the cube center and double is the
  // dimension of each side.
  void getAllFreeBoxes(
      std::vector<std::pair<StateVec, double> >* free_box_vector) const;
  void getAllOccupiedBoxes(std::vector<std::pair<StateVec, double> >*
                               occupied_box_vector) const;
  void getBox(const octomap::OcTreeKey& key,
              std::pair<StateVec, double>* box) const;
  void getFreeBoxesBoundingBox(
      const StateVec& position, const StateVec& bounding_box_size,
      std::vector<std::pair<StateVec, double> >* free_box_vector) const;
  void getOccupiedBoxesBoundingBox(
      const StateVec& position, const StateVec& bounding_box_size,
      std::vector<std::pair<StateVec, double> >* occupied_box_vector)
      const;

  virtual double getResolution() const;
  virtual double getSensorMaxRange() const;
  virtual StateVec getMapCenter() const;
  virtual StateVec getMapSize() const;
  virtual void getMapBounds(StateVec* min_bound,
                            StateVec* max_bound) const;

  // Collision checking with robot model. Implemented as a box with our own
  // implementation.
  virtual void setRobotSize(const StateVec& robot_size);
  virtual StateVec getRobotSize() const;
  virtual bool checkCollisionWithRobot(const StateVec& robot_position);
  // Checks a path (assumed to be time-ordered) for collision.
  // Sets the second input to the index at which the collision occurred.
  virtual bool checkPathForCollisionsWithRobot(
      const std::vector<StateVec>& robot_positions,
      size_t* collision_index);

  // Serialization and deserialization from ROS messages.
  bool getOctomapBinaryMsg(octomap_msgs::msg::Octomap* msg) const;
  bool getOctomapFullMsg(octomap_msgs::msg::Octomap* msg) const;
  // Clears the current octomap and replaces it with one from the message.
  void setOctomapFromMsg(const octomap_msgs::msg::Octomap& msg);
  // Loading and writing to disk.
  bool loadOctomapFromFile(const std::string& filename);
  bool writeOctomapToFile(const std::string& filename);

  // Helpers for publishing.
  void generateMarkerArray(const std::string& tf_frame,
                           visualization_msgs::msg::MarkerArray* occupied_nodes,
                           visualization_msgs::msg::MarkerArray* free_nodes);

  // Change detection -- when this is called, this resets the change detection
  // tracking within the map. So 2 consecutive calls will produce first the
  // change set, then nothing.
  // If not NULL, changed_states contains the new state of the node -- 1 is
  // occupied, 0 is free.
  // IMPORTANT NOTE: change_detection MUST be set to true in the parameters in
  // order for this to work!
  void getChangedPoints(std::vector<StateVec>* changed_points,
                        std::vector<bool>* changed_states);

  void coordToKey(const StateVec& coord, octomap::OcTreeKey* key) const;
  void keyToCoord(const octomap::OcTreeKey& key, StateVec* coord) const;

  std::shared_ptr<octomap::OcTree> octree_;

 protected:
  // Actual implementation for inserting disparity data.
  virtual void insertProjectedDisparityIntoMapImpl(
      const Transformation& sensor_to_world, const cv::Mat& projected_points);

  // Actual implementation for inserting pointclouds.
  virtual void insertPointcloudIntoMapImpl(
      const Transformation& T_G_sensor,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

  // Check if the node at the specified key has neighbors or not.
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  // Manually affect the probabilities of areas within a bounding box.
  void setLogOddsBoundingBox(const StateVec& position,
                             const StateVec& bounding_box_size,
                             double log_odds_value);

  void getAllBoxes(bool occupied_boxes,
                   std::vector<std::pair<StateVec, double> >* box_vector)
      const;
  void getBoxesBoundingBox(bool occupied_boxes, const StateVec& position,
                           const StateVec& bounding_box_size,
                           std::vector<std::pair<StateVec, double> >*
                               occupied_box_vector) const;

  // Helper functions for building up a map from sensor data.
  void castRay(const octomap::point3d& sensor_origin,
               const octomap::point3d& point, octomap::KeySet* free_cells,
               octomap::KeySet* occupied_cells);
  void updateOccupancy(octomap::KeySet* free_cells,
                       octomap::KeySet* occupied_cells);
  bool isValidPoint(const cv::Vec3f& point) const;

  void setOctomapFromBinaryMsg(const octomap_msgs::msg::Octomap& msg);
  void setOctomapFromFullMsg(const octomap_msgs::msg::Octomap& msg);

  double colorizeMapByHeight(double z, double min_z, double max_z) const;

  // Collision checking methods.
  bool checkSinglePoseCollision(const StateVec& robot_position) const;

  std_msgs::msg::ColorRGBA percentToColor(double h) const;

  OctomapParameters params_;

  // For collision checking.
  StateVec robot_size_;

  // Temporary variable for KeyRay since it resizes it to a HUGE value by
  // default. Thanks a lot to @xiaopenghuang for catching this.
  octomap::KeyRay key_ray_;

};

}  // namespace volumetric_mapping

#endif  // OCTOMAP_WORLD_OCTOMAP_WORLD_H_
