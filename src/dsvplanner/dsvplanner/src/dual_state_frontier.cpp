/**************************************************************************
dual_state_fontier.cpp
Implementation of dual_state frontier detection. Detect local and global
frontiers
to guide the extension of local and global graph.

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "dsvplanner/dual_state_frontier.h"

#include <misc_utils/misc_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace dsvplanner_ns
{
DualStateFrontier::DualStateFrontier(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                                     volumetric_mapping::OctomapManager* manager, OccupancyGrid* grid)
  : nh_(nh), nh_private_(nh_private)
{
  manager_ = manager;
  grid_ = grid;
  initialize();
}

DualStateFrontier::~DualStateFrontier()
{
}

bool DualStateFrontier::readParameters()
{
  nh_private_.getParam("/frontier/world_frame_id", world_frame_id_);
  nh_private_.getParam("/planner/odomSubTopic", sub_odom_topic_);
  nh_private_.getParam("/planner/terrainCloudSubTopic", sub_terrain_point_cloud_topic_);
  nh_private_.getParam("/frontier/sub_graph_points_topic", sub_graph_points_topic_);
  nh_private_.getParam("/frontier/pub_unknown_points_topic", pub_unknown_points_topic_);
  nh_private_.getParam("/frontier/pub_global_frontier_points_topic", pub_global_frontier_points_topic_);
  nh_private_.getParam("/frontier/pub_local_frontier_points_topic", pub_local_frontier_points_topic_);
  nh_private_.getParam("/frontier/kExecuteFrequency", kExecuteFrequency_);
  nh_private_.getParam("/frontier/kFrontierResolution", kFrontierResolution);
  nh_private_.getParam("/frontier/kFrontierFilterSize", kFrontierFilterSize);
  nh_private_.getParam("/frontier/kSearchRadius", kSearchRadius);
  nh_private_.getParam("/frontier/kSearchBoundingX", search_bounding[0]);
  nh_private_.getParam("/frontier/kSearchBoundingY", search_bounding[1]);
  nh_private_.getParam("/frontier/kSearchBoundingZ", search_bounding[2]);
  nh_private_.getParam("/frontier/kEffectiveUnknownNumAroundFrontier", kEffectiveUnknownNumAroundFrontier);
  nh_private_.getParam("/frontier/kFrontierNeighbourSearchRadius", kFrontierNeighbourSearchRadius);
  nh_private_.getParam("/frontier/kEliminateFrontiersAroundRobots", kEliminateFrontiersAroundRobots);
  nh_private_.getParam("/gb/kMaxXGlobal", kGlobalMaxX);
  nh_private_.getParam("/gb/kMaxYGlobal", kGlobalMaxY);
  nh_private_.getParam("/gb/kMaxZGlobal", kGlobalMaxZ);
  nh_private_.getParam("/gb/kMinXGlobal", kGlobalMinX);
  nh_private_.getParam("/gb/kMinYGlobal", kGlobalMinY);
  nh_private_.getParam("/gb/kMinZGlobal", kGlobalMinZ);
  nh_private_.getParam("/rm/kBoundX", robot_bounding[0]);
  nh_private_.getParam("/rm/kBoundY", robot_bounding[1]);
  nh_private_.getParam("/rm/kBoundZ", robot_bounding[2]);
  nh_private_.getParam("/rm/kSensorVertical", kSensorVerticalView);
  nh_private_.getParam("/rm/kSensorHorizontal", kSensorHorizontalView);
  nh_private_.getParam("/rm/kVehicleHeight", kVehicleHeight);
  nh_private_.getParam("/elevation/kTerrainVoxelSize", kTerrainVoxelSize);
  nh_private_.getParam("/elevation/kTerrainVoxelHalfWidth", kTerrainVoxelHalfWidth);
  nh_private_.getParam("/elevation/kTerrainVoxelWidth", kTerrainVoxelWidth);

  return true;
}

void DualStateFrontier::getUnknowPointcloudInBoundingBox(const StateVec& center, const StateVec& bounding_box_size)
{
  unknown_points_->clear();
  local_frontier_->clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_temp =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  const double epsilon = 0.001;  // Small offset to not hit boundary of nodes.
  StateVec epsilon_3d;
  epsilon_3d.setConstant(epsilon);

  // Determine correct center of voxel.
  const StateVec center_corrected(
      kFrontierResolution * std::floor(center.x() / kFrontierResolution) + kFrontierResolution / 2.0,
      kFrontierResolution * std::floor(center.y() / kFrontierResolution) + kFrontierResolution / 2.0, center.z());
  StateVec bbx_min = -bounding_box_size / 2 - epsilon_3d;
  StateVec bbx_max = bounding_box_size / 2 + epsilon_3d;

  for (double x_position = bbx_min.x(); x_position <= bbx_max.x(); x_position += kFrontierResolution)
  {
    for (double y_position = bbx_min.y(); y_position <= bbx_max.y(); y_position += kFrontierResolution)
    {
      double x = center_corrected[0] + x_position;
      double y = center_corrected[1] + y_position;
      double z = getZvalue(x_position, y_position);
      if (z >= 1000)
        continue;
      octomap::point3d point = octomap::point3d(x, y, z);
      octomap::OcTreeKey key = manager_->octree_->coordToKey(point);
      octomap::OcTreeNode* node = manager_->octree_->search(key);
      if (node == NULL)
      {
        unknown_points_->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        if (FrontierInBoundry(point) && frontierDetect(point))
        {
          local_frontier_temp->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }
      }
    }
  }
  pcl::VoxelGrid<pcl::PointXYZ> point_ds;
  point_ds.setLeafSize(kFrontierFilterSize, kFrontierFilterSize, kFrontierFilterSize);
  point_ds.setInputCloud(local_frontier_temp);
  point_ds.filter(*local_frontier_);
}

bool DualStateFrontier::frontierDetect(octomap::point3d point) const
{
  const double resolution = manager_->octree_->getResolution();
  bool xPositive = false, xNegative = false, yPositive = false, yNegative = false;
  bool effectiveFree = false;
  bool effectiveUnknown = false;
  int unknowCount = 0;
  octomap::OcTreeNode* node_inside;
  octomap::OcTreeNode* node_outside;
  octomap::OcTreeKey key_inside, key_outside;
  octomap::point3d surround_point_inside, surround_point_outside;
  surround_point_inside.x() = point.x();
  surround_point_inside.y() = point.y() - resolution;
  surround_point_inside.z() = point.z();
  key_inside = manager_->octree_->coordToKey(surround_point_inside);
  node_inside = manager_->octree_->search(key_inside);
  surround_point_outside.x() = point.x();
  surround_point_outside.y() = point.y() - 2 * resolution;
  surround_point_outside.z() = point.z();
  key_outside = manager_->octree_->coordToKey(surround_point_outside);
  node_outside = manager_->octree_->search(key_outside);
  if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
  {
    yNegative = true;
  }
  else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
  {
    return false;
  }
  else if (node_inside == NULL)
  {
    unknowCount++;
  }
  surround_point_inside.x() = point.x();
  surround_point_inside.y() = point.y() + resolution;
  surround_point_inside.z() = point.z();
  key_inside = manager_->octree_->coordToKey(surround_point_inside);
  node_inside = manager_->octree_->search(key_inside);
  surround_point_outside.x() = point.x();
  surround_point_outside.y() = point.y() + 2 * resolution;
  surround_point_outside.z() = point.z();
  key_outside = manager_->octree_->coordToKey(surround_point_outside);
  node_outside = manager_->octree_->search(key_outside);
  if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
  {
    yPositive = true;
  }
  else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
  {
    return false;
  }
  else if (node_inside == NULL)
  {
    unknowCount++;
  }
  surround_point_inside.x() = point.x() - resolution;
  surround_point_inside.y() = point.y();
  surround_point_inside.z() = point.z();
  key_inside = manager_->octree_->coordToKey(surround_point_inside);
  node_inside = manager_->octree_->search(key_inside);
  surround_point_outside.x() = point.x() - 2 * resolution;
  surround_point_outside.y() = point.y();
  surround_point_outside.z() = point.z();
  key_outside = manager_->octree_->coordToKey(surround_point_outside);
  node_outside = manager_->octree_->search(key_outside);
  if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
  {
    xNegative = true;
  }
  else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
  {
    return false;
  }
  else if (node_inside == NULL)
  {
    unknowCount++;
  }
  surround_point_inside.x() = point.x() + resolution;
  surround_point_inside.y() = point.y();
  surround_point_inside.z() = point.z();
  key_inside = manager_->octree_->coordToKey(surround_point_inside);
  node_inside = manager_->octree_->search(key_inside);
  surround_point_outside.x() = point.x() + 2 * resolution;
  surround_point_outside.y() = point.y();
  surround_point_outside.z() = point.z();
  key_outside = manager_->octree_->coordToKey(surround_point_outside);
  node_outside = manager_->octree_->search(key_outside);
  if (node_inside != NULL && (!manager_->octree_->isNodeOccupied(node_inside)))
  {
    xPositive = true;
  }
  else if ((node_inside != NULL && manager_->octree_->isNodeOccupied(node_inside)))
  {
    return false;
  }
  else if (node_inside == NULL)
  {
    unknowCount++;
  }
  effectiveFree = xPositive || xNegative || yPositive || yNegative;
  effectiveUnknown = unknowCount >= kEffectiveUnknownNumAroundFrontier;
  return (effectiveFree && effectiveUnknown);
}

bool DualStateFrontier::FrontierInBoundry(octomap::point3d point) const
{
  if (boundaryLoaded_)
  {
    geometry_msgs::Point node_point;
    node_point.x = point.x();
    node_point.y = point.y();
    node_point.z = point.z();
    if (!misc_utils_ns::PointInPolygon(node_point, boundary_polygon_))
    {
      return false;
    }
  }
  else
  {
    if (point.x() > kGlobalMaxX)
      return false;
    else if (point.y() > kGlobalMaxY)
      return false;
    else if (point.x() < kGlobalMinX)
      return false;
    else if (point.y() < kGlobalMinY)
      return false;
  }
  if (point.z() > kGlobalMaxZ)
    return false;
  if (point.z() < kGlobalMinZ)
    return false;
  else
    return true;
}

void DualStateFrontier::updateToCleanFrontier(pcl::PointXYZ point)
{
  cleanedFrontier_->points.push_back(point);
}

bool DualStateFrontier::isCleanedFrontier(pcl::PointXYZ point)
{
  if (cleanedFrontier_->points.size() > 0)
  {
    for (int i = 0; i < cleanedFrontier_->points.size(); i++)
    {
      double dist = sqrt((point.x - cleanedFrontier_->points[i].x) * (point.x - cleanedFrontier_->points[i].x) +
                         (point.y - cleanedFrontier_->points[i].y) * (point.y - cleanedFrontier_->points[i].y) +
                         (point.z - cleanedFrontier_->points[i].z) * (point.x - cleanedFrontier_->points[i].z));
      if (dist < 3)
      {
        return true;
      }
    }
  }
  return false;
}

bool DualStateFrontier::inSensorRangeofGraphPoints(StateVec point)
{
  pcl::PointXYZ check_point;
  check_point.x = point[0];
  check_point.y = point[1];
  check_point.z = point[2];
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchDist;
  if (graphPoints_->points.size() > 0)
  {
    kdtree_->setInputCloud(graphPoints_);
    kdtree_->radiusSearch(check_point, kSearchRadius, pointSearchInd, pointSearchDist);
    for (int i = 0; i < pointSearchInd.size(); i++)
    {
      StateVec node_point(graphPoints_->points[pointSearchInd[i]].x, graphPoints_->points[pointSearchInd[i]].y,
                          graphPoints_->points[pointSearchInd[i]].z);
      StateVec dir = point - node_point;
      // Skip if distance is too large
      double rangeSq = pow(kSearchRadius, 2.0);
      if (dir.transpose().dot(dir) > rangeSq)
      {
        continue;
      }
      bool insideAFieldOfView = false;
      if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * kSensorVerticalView / 360)))
      {
        insideAFieldOfView = true;
      }
      if (!insideAFieldOfView)
      {
        continue;
      }
      if (manager_->CellStatus::kOccupied != manager_->getVisibility(node_point, point, false) &&
          !grid_->collisionCheckByTerrainWithVector(node_point, point))
      {
        return true;
      }
    }
    return false;
  }
  return false;
}

bool DualStateFrontier::inSensorRangeofRobot(StateVec point)
{
  pcl::PointXYZ check_point;
  check_point.x = point[0];
  check_point.y = point[1];
  check_point.z = point[2];
  StateVec dir = point - robot_position_;
  // Skip if distance is too large
  double rangeSq = pow(kSearchRadius, 2.0);
  if (dir.transpose().dot(dir) > rangeSq)
  {
    return false;
  }
  bool insideAFieldOfView = false;
  if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * kSensorVerticalView / 360)))
  {
    insideAFieldOfView = true;
  }
  if (!insideAFieldOfView)
  {
    return false;
  }
  if (manager_->CellStatus::kFree != manager_->getVisibility(robot_position_, point, false))
  {
    return false;
  }
  return true;
}

void DualStateFrontier::localFrontierUpdate(StateVec& center)
{
  local_frontier_pcl_->clear();
  StateVec checkedPoint;

  for (int i = 0; i < local_frontier_->size(); i++)
  {
    checkedPoint.x() = local_frontier_->points[i].x;
    checkedPoint.y() = local_frontier_->points[i].y;
    checkedPoint.z() = local_frontier_->points[i].z;

    if (!(kEliminateFrontiersAroundRobots && inSensorRangeofRobot(checkedPoint)) &&
        ((manager_->CellStatus::kOccupied != manager_->getVisibility(center, checkedPoint, false) &&
          !grid_->collisionCheckByTerrainWithVector(center, checkedPoint)) ||
         (!planner_status_ && inSensorRangeofGraphPoints(checkedPoint))))
    {
      local_frontier_pcl_->points.push_back(local_frontier_->points[i]);
      global_frontier_->points.push_back(local_frontier_->points[i]);
    }
  }
  local_frontier_->clear();
  pcl::VoxelGrid<pcl::PointXYZ> point_ds;
  point_ds.setLeafSize(kFrontierFilterSize, kFrontierFilterSize, kFrontierFilterSize);
  point_ds.setInputCloud(local_frontier_pcl_);
  point_ds.filter(*local_frontier_);
}

void DualStateFrontier::gloabalFrontierUpdate()
{
  global_frontier_pcl_->clear();

  int size = global_frontier_->points.size();
  octomap::OcTreeNode* node;
  octomap::OcTreeKey key;
  octomap::point3d point;
  StateVec checkedPoint;
  for (int i = 0; i < size; i++)
  {
    point.x() = global_frontier_->points[i].x;
    point.y() = global_frontier_->points[i].y;
    point.z() = global_frontier_->points[i].z;
    if (isCleanedFrontier(global_frontier_->points[i]))
    {
      continue;
    }
    checkedPoint.x() = point.x();
    checkedPoint.y() = point.y();
    checkedPoint.z() = point.z();
    key = manager_->octree_->coordToKey(point);
    node = manager_->octree_->search(key);
    if (node == NULL && frontierDetect(point))
    {
      global_frontier_pcl_->points.push_back(global_frontier_->points[i]);
    }
  }
  global_frontier_->clear();
  pcl::VoxelGrid<pcl::PointXYZ> point_ds_;
  point_ds_.setLeafSize(kFrontierFilterSize, kFrontierFilterSize, kFrontierFilterSize);
  point_ds_.setInputCloud(global_frontier_pcl_);
  point_ds_.filter(*global_frontier_);
}

void DualStateFrontier::globalFrontiersNeighbourCheck()
{
  global_frontier_pcl_->clear();

  int size = global_frontier_->points.size();
  pcl::PointXYZ p1;
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchDist;
  if (size > 0)
  {
    global_frontiers_kdtree_->setInputCloud(global_frontier_);
    for (int i = 0; i < size; i++)
    {
      p1 = global_frontier_->points[i];
      pointSearchInd.clear();
      pointSearchDist.clear();
      global_frontiers_kdtree_->radiusSearch(p1, kFrontierNeighbourSearchRadius, pointSearchInd, pointSearchDist);
      if (pointSearchInd.size() > 1)
        global_frontier_pcl_->points.push_back(p1);
    }
  }
  global_frontier_->clear();
  *global_frontier_ = *global_frontier_pcl_;
}

void DualStateFrontier::cleanAllUselessFrontiers()
{
  global_frontier_->clear();
  local_frontier_->clear();
  publishFrontiers();
}

void DualStateFrontier::getFrontiers()
{
  getUnknowPointcloudInBoundingBox(robot_position_, search_bounding);
  localFrontierUpdate(robot_position_);
  gloabalFrontierUpdate();
  globalFrontiersNeighbourCheck();
  publishFrontiers();
}

void DualStateFrontier::publishFrontiers()
{
  sensor_msgs::PointCloud2 unknown_pcl, local_frontier_pcl, global_frontier_pcl;
  pcl::toROSMsg(*unknown_points_, unknown_pcl);
  pcl::toROSMsg(*global_frontier_, global_frontier_pcl);
  pcl::toROSMsg(*local_frontier_, local_frontier_pcl);
  unknown_pcl.header.frame_id = world_frame_id_;
  global_frontier_pcl.header.frame_id = world_frame_id_;
  local_frontier_pcl.header.frame_id = world_frame_id_;
  unknown_points_pub_.publish(unknown_pcl);
  global_frontier_points_pub_.publish(global_frontier_pcl);
  local_frontier_points_pub_.publish(local_frontier_pcl);
}

void DualStateFrontier::terrainCloudAndOdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                                    const sensor_msgs::PointCloud2::ConstPtr& terrain_msg)
{
  robot_position_[0] = odom_msg->pose.pose.position.x;
  robot_position_[1] = odom_msg->pose.pose.position.y;
  robot_position_[2] = odom_msg->pose.pose.position.z;
  terrain_cloud_->clear();
  terrain_cloud_ds->clear();
  terrain_elev_cloud_->clear();
  terrain_voxel_points_num_.clear();
  terrain_voxel_min_elev_.clear();
  terrain_voxel_max_elev_.clear();
  terrain_voxel_points_num_.resize(kTerrainVoxelWidth * kTerrainVoxelWidth);
  terrain_voxel_min_elev_.resize(kTerrainVoxelWidth * kTerrainVoxelWidth, 1000);
  terrain_voxel_max_elev_.resize(kTerrainVoxelWidth * kTerrainVoxelWidth, -1000);
  pcl::fromROSMsg(*terrain_msg, *terrain_cloud_);

  pcl::VoxelGrid<pcl::PointXYZI> point_ds;
  point_ds.setLeafSize(0.3, 0.3, 0.3);
  point_ds.setInputCloud(terrain_cloud_);
  point_ds.filter(*terrain_cloud_ds);

  updateTerrainMinElevation();
  updateTerrainElevationForKnown();
  updateTerrainElevationForUnknow();

  sensor_msgs::PointCloud2 elevVoxel2;
  pcl::toROSMsg(*terrain_elev_cloud_, elevVoxel2);
  elevVoxel2.header.stamp = ros::Time::now();
  elevVoxel2.header.frame_id = "/map";
  terrain_elev_cloud_pub_.publish(elevVoxel2);
}

void DualStateFrontier::updateTerrainMinElevation()
{
  pcl::PointXYZI point;
  int terrainCloudSize = terrain_cloud_ds->points.size();
  for (int i = 0; i < terrainCloudSize; i++)
  {
    point.x = terrain_cloud_ds->points[i].x;
    point.y = terrain_cloud_ds->points[i].y;
    point.z = terrain_cloud_ds->points[i].z;
    point.intensity = terrain_cloud_ds->points[i].intensity;
    int indX = int((point.x - robot_position_[0] + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
    int indY = int((point.y - robot_position_[1] + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
    if (point.x - robot_position_[0] + kTerrainVoxelSize / 2 < 0)
      indX--;
    if (point.y - robot_position_[1] + kTerrainVoxelSize / 2 < 0)
      indY--;
    if (indX > kTerrainVoxelWidth - 1)
      indX = kTerrainVoxelWidth - 1;
    if (indX < 0)
      indX = 0;
    if (indY > kTerrainVoxelWidth - 1)
      indY = kTerrainVoxelWidth - 1;
    if (indY < 0)
      indY = 0;
    int indVoxel = kTerrainVoxelWidth * indX + indY;

    terrain_voxel_points_num_[indVoxel]++;
    if (point.z < terrain_voxel_min_elev_[indVoxel])
      terrain_voxel_min_elev_[indVoxel] = point.z;
    if (point.z > terrain_voxel_max_elev_[indVoxel])
      terrain_voxel_max_elev_[indVoxel] = point.z;
    for (int dX = -1; dX <= 1; dX = dX + 2)
    {
      for (int dY = -1; dY <= 1; dY = dY + 2)
      {
        if (indX + dX >= 0 && indX + dX < kTerrainVoxelWidth && indY + dY >= 0 && indY + dY < kTerrainVoxelWidth)
        {
          terrain_voxel_points_num_[kTerrainVoxelWidth * (indX + dX) + indY + dY]++;
          if (point.z < terrain_voxel_min_elev_[kTerrainVoxelWidth * (indX + dX) + indY + dY])
            terrain_voxel_min_elev_[kTerrainVoxelWidth * (indX + dX) + indY + dY] = point.z;
        }
      }
    }
  }
}

void DualStateFrontier::updateTerrainElevationForKnown()
{
  pcl::PointXYZI point;
  for (int i = 0; i < kTerrainVoxelWidth * kTerrainVoxelWidth; i++)
  {
    if (terrain_voxel_points_num_[i] > 0)
    {
      if (terrain_voxel_max_elev_[i] - terrain_voxel_min_elev_[i] >= 0.4)
        terrain_voxel_elev_[i] = 1000;  // set a high value to untraversable
                                        // voxel
      else
        terrain_voxel_elev_[i] = terrain_voxel_min_elev_[i];

      int indX = int(i / kTerrainVoxelWidth);
      int indY = i % kTerrainVoxelWidth;
      if (indX - kTerrainVoxelHalfWidth < 0)
      {
        indX++;
      }
      if (indY - kTerrainVoxelWidth < 0)
      {
        indY++;
      }
      point.x = (indX - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[0];
      point.y = (indY - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[1];
      point.z = 0;
      point.intensity = terrain_voxel_elev_[i];
      terrain_elev_cloud_->push_back(point);
    }
  }
}

void DualStateFrontier::updateTerrainElevationForUnknow()
{
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  kdtree.setInputCloud(terrain_elev_cloud_);

  pcl::PointXYZI point;
  if (terrain_voxel_points_num_[0] <= 0)
  {
    terrain_voxel_elev_[0] = robot_position_[2] - kVehicleHeight;
  }
  for (int i = 1; i < kTerrainVoxelWidth * kTerrainVoxelWidth; i++)
  {
    if (terrain_voxel_points_num_[i] <= 0)
    {
      int indX = int(i / kTerrainVoxelWidth);
      int indY = i % kTerrainVoxelWidth;
      point.x = (indX - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[0];
      point.y = (indY - kTerrainVoxelHalfWidth) * kTerrainVoxelSize - kTerrainVoxelSize / 2 + robot_position_[1];
      point.z = 0;

      if (terrain_elev_cloud_->points.size() > 0)
      {
        if (kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
          point.intensity = terrain_elev_cloud_->points[pointIdxNKNSearch[0]].intensity;
          terrain_voxel_elev_[i] = point.intensity;
        }
        terrain_elev_cloud_->push_back(point);
      }
    }
  }
}

void DualStateFrontier::graphPointsCallback(const sensor_msgs::PointCloud2& graph_msg)
{
  graphPoints_->clear();
  pcl::fromROSMsg(graph_msg, *graphPoints_);
}

double DualStateFrontier::getZvalue(double x_position, double y_position)
{
  int indX = int((x_position + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
  int indY = int((y_position + kTerrainVoxelSize / 2) / kTerrainVoxelSize) + kTerrainVoxelHalfWidth;
  if (x_position + kTerrainVoxelSize / 2 < 0)
    indX--;
  if (y_position + kTerrainVoxelSize / 2 < 0)
    indY--;
  if (indX > kTerrainVoxelWidth - 1)
    indX = kTerrainVoxelWidth - 1;
  if (indX < 0)
    indX = 0;
  if (indY > kTerrainVoxelWidth - 1)
    indY = kTerrainVoxelWidth - 1;
  if (indY < 0)
    indY = 0;
  return terrain_voxel_elev_[kTerrainVoxelWidth * indX + indY] + kVehicleHeight;
}

std::vector<double> DualStateFrontier::getTerrainVoxelElev()
{
  return terrain_voxel_elev_;
}

void DualStateFrontier::setPlannerStatus(bool status)
{
  planner_status_ = status;
}

void DualStateFrontier::setBoundary(const geometry_msgs::PolygonStamped& boundary)
{
  boundary_polygon_ = boundary.polygon;
  boundaryLoaded_ = true;
}

bool DualStateFrontier::initialize()
{
  // Read in parameters
  if (!readParameters())
    return false;

  // Initialize subscriber
  graph_points_sub_ = nh_.subscribe(sub_graph_points_topic_, 1, &DualStateFrontier::graphPointsCallback, this);
  odom_sub_.subscribe(nh_, sub_odom_topic_, 1);
  terrain_point_cloud_sub_.subscribe(nh_, sub_terrain_point_cloud_topic_, 1);
  sync_.reset(new Sync(syncPolicy(10), odom_sub_, terrain_point_cloud_sub_));
  sync_->registerCallback(boost::bind(&DualStateFrontier::terrainCloudAndOdomCallback, this, _1, _2));

  unknown_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_unknown_points_topic_, 1);
  global_frontier_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_global_frontier_points_topic_, 1);
  local_frontier_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_local_frontier_points_topic_, 1);
  terrain_elev_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/elevation_map", 1);

  if (kExecuteFrequency_ > 0.0)
  {
    executeTimer_ = nh_private_.createTimer(ros::Duration(1.0 / kExecuteFrequency_), &DualStateFrontier::execute, this);
  }
  for (int i = 0; i < kTerrainVoxelWidth * kTerrainVoxelWidth; i++)
  {
    terrain_voxel_elev_.push_back(robot_position_.z());
    terrain_voxel_points_num_.push_back(0);
    terrain_voxel_min_elev_.push_back(1000);
    terrain_voxel_max_elev_.push_back(-1000);
  }

  boundaryLoaded_ = false;

  ROS_INFO("Successfully launched DualStateFrontier node");

  return true;
}

void DualStateFrontier::execute(const ros::TimerEvent& e)
{
  getFrontiers();
}
}
