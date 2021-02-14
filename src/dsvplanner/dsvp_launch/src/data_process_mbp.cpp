
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
#include <visualization_msgs/Marker.h>
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
ros::Time current_local_plan_time;
ros::Time current_global_plan_time;
double timeInterval = 0;
double localPlanTime = 0;
double globalPlanTime = 0;
double travelDist = 0;
int voxelNum = 0;

std::string time_volume_name;
std::string time_dist_name;
std::string time_plantime_name;
std::string dist_plantime_name;
std::string time_localplan_name;
std::string dist_localplan_name;

std::string planner_name = "drrt";

PointCloud<PointXYZI>::Ptr keyposecloud(new PointCloud<PointXYZI>);
PointCloud<PointXYZI>::Ptr keyposecloudPassFiltered(new PointCloud<PointXYZI>);
PointCloud<PointXYZI>::Ptr mapcloud(new PointCloud<PointXYZI>);
PointCloud<PointXYZI>::Ptr mapcloudDS(new PointCloud<PointXYZI>);
VoxelGrid<PointXYZI> point_ds;

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
void write_to_volume_file(double timeInt, int volume)
{
  out1 << timeInt << " " << volume << endl;
}

void write_to_dist_file(double timeInt, double dist)
{
  out2 << timeInt << " " << dist << endl;
}

void write_to_time_globalplantime_file(double timeInt, double plantimeInt)
{
  out3 << timeInt << " " << plantimeInt << endl;
}

void write_to_dist_globalplantime_file(double dist, double plantimeInt)
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

int pointvolume(PointCloud<PointXYZI>::Ptr cloud)
{
  *mapcloud += *cloud;
  point_ds.setLeafSize(0.5, 0.5, 0.5);
  point_ds.setInputCloud(mapcloud);
  point_ds.filter(*mapcloudDS);
  pcl::toROSMsg(*mapcloudDS, dscloud);
  dscloud.header.frame_id = "/map";
  mapcloudDS_pub.publish(dscloud);
  return mapcloudDS->points.size();
}

void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  keyposecloud->clear();
  if (first_iteration == false)
  {
    current_time = cloud_msg->header.stamp;
    timeInterval = (current_time - time_start).toSec();

    pcl::fromROSMsg(*cloud_msg, *keyposecloud);
    pcl::PointXYZI p1;
    for (int i = 0; i < keyposecloud->points.size(); i++)
    {
      p1 = keyposecloud->points[i];
      keyposecloud->points[i].x = p1.z;
      keyposecloud->points[i].y = p1.x;
      keyposecloud->points[i].z = p1.y;
    }

    keyposecloudPassFiltered->clear();
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    boxFilter.setInputCloud(keyposecloud);
    //    boxFilter.setMin(Eigen::Vector4f(-60, -75, -20, 1.0));
    //    boxFilter.setMax(Eigen::Vector4f(35, 28, 10, 1.0));
    boxFilter.setMin(Eigen::Vector4f(-30, -40, -3, 1.0));
    boxFilter.setMax(Eigen::Vector4f(65, 85, 5, 1.0));
    boxFilter.filter(*keyposecloudPassFiltered);

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
    current_time = msg->header.stamp;
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

void LocalPlanTimeCallback(const geometry_msgs::PointStamped::ConstPtr& local_time_msg)
{
  if (first_iteration)
  {
    time_start = local_time_msg->header.stamp;
    mapcloud->clear();
    travelDist = 0;
    first_iteration = false;
  }
  else
  {
    current_local_plan_time = local_time_msg->header.stamp;
    localPlanTime = local_time_msg->point.x;
    timeInterval = (current_local_plan_time - time_start).toSec();
    write_to_time_localplantime_file(timeInterval, localPlanTime);
    write_to_dist_localplantime_file(travelDist, localPlanTime);
  }
}

void GlobalPlanTimeCallback(const geometry_msgs::PointStamped::ConstPtr& global_time_msg)
{
  if (first_iteration == false)
  {
    current_global_plan_time = global_time_msg->header.stamp;
    globalPlanTime = global_time_msg->point.x;
    timeInterval = (current_global_plan_time - time_start).toSec();
    write_to_time_globalplantime_file(timeInterval, globalPlanTime);
    write_to_dist_globalplantime_file(travelDist, globalPlanTime);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_with_lp");
  ros::NodeHandle nh;
  ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, OdometryCallback);
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered2", 1, CloudCallback);

  ros::Subscriber local_plan_time_sub =
      nh.subscribe<geometry_msgs::PointStamped>("/plan_time", 1, LocalPlanTimeCallback);
  ros::Subscriber global_plan_time_sub =
      nh.subscribe<geometry_msgs::PointStamped>("/effective_plan_time", 1, GlobalPlanTimeCallback);

  mapcloudDS_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloudds", 1);

  ROS_INFO("Started Timing");

  planner_name = argv[argc - 1];
  time_volume_name = ros::package::getPath("dsvplanner_launch") + "/data/" + planner_name + "_time_volume.txt";
  time_dist_name = ros::package::getPath("dsvplanner_launch") + "/data/" + planner_name + "_time_dist.txt";
  time_plantime_name = ros::package::getPath("dsvplanner_launch") + "/data/" + planner_name + "_time_plantime.txt";
  dist_plantime_name = ros::package::getPath("dsvplanner_launch") + "/data/" + planner_name + "_dist_plantime.txt";
  time_localplan_name = ros::package::getPath("dsvplanner_launch") + "/data/" + planner_name + "_time_"
                                                                                               "localplantime.txt";
  dist_localplan_name = ros::package::getPath("dsvplanner_launch") + "/data/" + planner_name + "_dist_"
                                                                                               "localplantime.txt";

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
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return -1;
}
