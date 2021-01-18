/*
data_collect_gbp.cpp
planner data processing for gbplanner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <fstream>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf/transform_datatypes.h>

using namespace std;
using namespace pcl;
ros::Time time_start;
ros::Time current_time;
ros::Time plan_end;
ros::Time localplan_end_prev;
ros::Time localplan_end;
ros::Time globalplan_end;
ros::Time wp_time;
ros::Time prev_wp;

double timeInterval = 0;
double plantimeInterval = 0;
double travelDist = 0;
double plantime = 0;
int voxelNum = 0;
std::string time_volume_name;
std::string time_dist_name;
std::string time_plantime_name;
std::string dist_plantime_name;
std::string time_localplan_name;
std::string dist_localplan_name;

PointCloud<PointXYZ>::Ptr keyposecloud(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr keyposecloudPassFiltered(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr mapcloud(new PointCloud<PointXYZ>);
PointCloud<PointXYZ>::Ptr mapcloudDS(new PointCloud<PointXYZ>);
sensor_msgs::PointCloud2 dscloud;

nav_msgs::Odometry previous_odom;
nav_msgs::Odometry current_odom;

bool first_iteration = true;

ofstream time_volume_file;
ofstream time_dist_file;
ofstream time_plantime_file;
ofstream dist_plantime_file;
ofstream time_localplantime_file;
ofstream dist_localplantime_file;

std::ostream& out1 = time_volume_file;
std::ostream& out2 = time_dist_file;
std::ostream& out3 = time_plantime_file;
std::ostream& out4 = dist_plantime_file;
std::ostream& out5 = time_localplantime_file;
std::ostream& out6 = dist_localplantime_file;

ros::Publisher mapcloudDS_pub;

int pointvolume(PointCloud<PointXYZ>::Ptr cloud)
{
  for (int i = 0; i < cloud->points.size(); i++)
  {
    mapcloud->points.push_back(cloud->points[i]);
  }
  pcl::VoxelGrid<pcl::PointXYZ> point_ds;
  point_ds.setLeafSize(0.1, 0.1, 0.1);
  point_ds.setInputCloud(mapcloud);
  point_ds.filter(*mapcloudDS);
  pcl::toROSMsg(*cloud, dscloud);
  dscloud.header.frame_id = "/map";
  mapcloudDS_pub.publish(dscloud);
  return mapcloudDS->points.size();
}

void write_to_volume_file(double timeInt, int volume)
{
  out1 << timeInt << " " << volume << endl;
}

void write_to_dist_file(double timeInt, double dist)
{
  out2 << timeInt << " " << dist << endl;
}

void write_to_time_plantime_file(double timeInt, double plantimeInt)
{
  out3 << timeInt << " " << plantimeInt << endl;
}

void write_to_dist_plantime_file(double dist, double plantimeInt)
{
  out4 << dist << " " << plantimeInt << endl;
}

void write_to_time_localplantime_file(double timeInt, double plantimeInt)
{
  out5 << timeInt << " " << plantimeInt << endl;
}

void write_to_dist_localplantime_file(double dist, double plantimeInt)
{
  out6 << dist << " " << plantimeInt << endl;
}

void keyposeCloudToMap(const nav_msgs::Odometry::ConstPtr& keypose_msg,
                       const sensor_msgs::PointCloud2ConstPtr& keypose_cloud_msg)
{
  // get position and orientation of the robot in keypose frame
  float robot_x = (float)keypose_msg->pose.pose.position.x;
  float robot_y = (float)keypose_msg->pose.pose.position.y;
  float robot_z = (float)keypose_msg->pose.pose.position.z;

  // std::cout<<"x="<<robot_x<<"y="<<robot_y<<"z="<<robot_z<<std::endl;

  tf::Quaternion tf_q(keypose_msg->pose.pose.orientation.x, keypose_msg->pose.pose.orientation.y,
                      keypose_msg->pose.pose.orientation.z, keypose_msg->pose.pose.orientation.w);
  tf::Matrix3x3 tf_m(tf_q);
  double robot_roll, robot_pitch, robot_yaw;
  tf_m.getRPY(robot_roll, robot_pitch, robot_yaw);

  float sin_roll = (float)sin(robot_roll);
  float cos_roll = (float)cos(robot_roll);
  float sin_pitch = (float)sin(robot_pitch);
  float cos_pitch = (float)cos(robot_pitch);
  float sin_yaw = (float)sin(robot_yaw);
  float cos_yaw = (float)cos(robot_yaw);

  /**get keypose cloud in map**/
  pcl::fromROSMsg(*keypose_cloud_msg, *keyposecloud);
  // transfer from PointXYZ to PointXYZI
  keyposecloudPassFiltered->clear();
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setInputCloud(keyposecloud);
  boxFilter.setMin(Eigen::Vector4f(-15, -20, -15, 1.0));
  boxFilter.setMax(Eigen::Vector4f(15, 10, 15, 1.0));
  boxFilter.filter(*keyposecloudPassFiltered);

  size_t keypose_cloud_size = keyposecloudPassFiltered->size();
  for (int i = 0; i < keypose_cloud_size; i++)
  {
    PointXYZ point;
    point = keyposecloudPassFiltered->points[i];

    // from keypose to map_rot
    float x1 = point.x;
    float y1 = point.y;
    float z1 = point.z;

    float x2 = x1;
    float y2 = y1 * cos_roll - z1 * sin_roll;
    float z2 = y1 * sin_roll + z1 * cos_roll;

    float x3 = x2 * cos_pitch + z2 * sin_pitch;
    float y3 = y2;
    float z3 = -x2 * sin_pitch + z2 * cos_pitch;

    float x4 = x3 * cos_yaw - y3 * sin_yaw;
    float y4 = x3 * sin_yaw + y3 * cos_yaw;
    float z4 = z3;

    float x5 = x4 + robot_x;
    float y5 = y4 + robot_y;
    float z5 = z4 + robot_z;

    // To map frame
    point.x = z5;
    point.y = x5;
    point.z = y5;

    keyposecloudPassFiltered->points[i].x = point.x;
    keyposecloudPassFiltered->points[i].y = point.y;
    keyposecloudPassFiltered->points[i].z = point.z;
  }
}

void keyPoseAndKeyPoseCloudCallback(const nav_msgs::Odometry::ConstPtr& keypose_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& keypose_cloud_msg)
{
  keyposecloud->clear();
  if (first_iteration == false)
  {
    current_time = ros::Time::now();
    timeInterval = (current_time - time_start).toSec();
    keyposeCloudToMap(keypose_msg, keypose_cloud_msg);
    voxelNum = pointvolume(keyposecloudPassFiltered);
    write_to_volume_file(timeInterval, voxelNum);
  }
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  previous_odom = current_odom;
  current_odom = *msg;
  if (first_iteration == false)
  {
    current_time = ros::Time::now();
    timeInterval = (current_time - time_start).toSec();
    double currentdist = sqrt((current_odom.pose.pose.position.x - previous_odom.pose.pose.position.x) *
                                  (current_odom.pose.pose.position.x - previous_odom.pose.pose.position.x) +
                              (current_odom.pose.pose.position.y - previous_odom.pose.pose.position.y) *
                                  (current_odom.pose.pose.position.y - previous_odom.pose.pose.position.y) +
                              (current_odom.pose.pose.position.z - previous_odom.pose.pose.position.z) *
                                  (current_odom.pose.pose.position.z - previous_odom.pose.pose.position.z));
    travelDist = travelDist + currentdist;
    write_to_dist_file(timeInterval, travelDist);
  }
}

void planTimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
  if (first_iteration)
  {
    localplan_end_prev = ros::Time::now();
    localplan_end = ros::Time::now();
    globalplan_end = ros::Time::now();
    plan_end = ros::Time::now();
    time_start = ros::Time::now();
    mapcloud->clear();
    travelDist = 0;
    first_iteration = false;
  }
  else
  {
    localplan_end_prev = localplan_end;
    localplan_end = ros::Time::now();
    if (plan_end < localplan_end && plan_end > localplan_end_prev)
    {
      globalplan_end = ros::Time::now();
      plantime = msg->data;
    }
  }
}

void wayPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  prev_wp = wp_time;
  wp_time = ros::Time::now();
  if (first_iteration == false && prev_wp < localplan_end)
  {
    current_time = ros::Time::now();
    plan_end = ros::Time::now();
    plantimeInterval = (plan_end - globalplan_end).toSec() + plantime;
    timeInterval = (current_time - time_start).toSec();
    write_to_time_plantime_file(timeInterval, plantimeInterval);
    write_to_dist_plantime_file(travelDist, plantimeInterval);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_with_lp");
  ros::NodeHandle nh;
  ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, OdometryCallback);
  ros::Subscriber plan_time_sub = nh.subscribe<std_msgs::Float32>("/plan_time", 2, planTimeCallback);
  ros::Subscriber gp_statur_sub = nh.subscribe<geometry_msgs::PointStamped>("/way_point", 5, wayPointCallback);

  message_filters::Subscriber<nav_msgs::Odometry> keypose_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> keypose_cloud_sub;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

  keypose_sub.subscribe(nh, "/key_pose_to_map", 1);
  keypose_cloud_sub.subscribe(nh, "/sensor_scan", 1);
  sync_.reset(new Sync(syncPolicy(10), keypose_sub, keypose_cloud_sub));
  sync_->registerCallback(boost::bind(keyPoseAndKeyPoseCloudCallback, _1, _2));

  mapcloudDS_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloudds", 1);

  ROS_INFO("Started Timing");

  time_volume_name = ros::package::getPath("dsvplanner_launch") + "/data/gbp_cic_time_volume.txt";
  time_dist_name = ros::package::getPath("dsvplanner_launch") + "/data/gbp_cic_time_dist.txt";
  time_plantime_name = ros::package::getPath("dsvplanner_launch") + "/data/gbp_cic_time_plantime.txt";
  dist_plantime_name = ros::package::getPath("dsvplanner_launch") + "/data/gbp_cic_dist_plantime.txt";
  time_localplan_name = ros::package::getPath("dsvplanner_launch") + "/data/"
                                                                     "gbp_cic_time_localplantime.txt";
  dist_localplan_name = ros::package::getPath("dsvplanner_launch") + "/data/"
                                                                     "gbp_cic_dist_localplantime.txt";

  time_volume_file.open(time_volume_name);
  time_dist_file.open(time_dist_name);
  time_plantime_file.open(time_plantime_name);
  dist_plantime_file.open(dist_plantime_name);
  time_localplantime_file.open(time_localplan_name);
  dist_localplantime_file.open(dist_localplan_name);

  time_volume_file.app;
  time_dist_file.app;
  time_plantime_file.app;
  dist_plantime_file.app;
  time_localplantime_file.app;
  dist_localplantime_file.app;

  out1 << "time volume" << endl;
  out2 << "time dist" << endl;
  out3 << "time plan_Interval" << endl;
  out4 << "dist plan_Interval" << endl;
  out5 << "time localplan_Interval" << endl;
  out6 << "dist localplan_Interval" << endl;

  // Start planning: The planner is called and the computed path sent to the controller.
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return -1;
}
