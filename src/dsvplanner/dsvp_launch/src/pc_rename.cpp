#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

sensor_msgs::PointCloud2 renamed_pc;
std::string sensor_frame = "sensor";
pcl::PointCloud<pcl::PointXYZ> PointClouds;
ros::Publisher renamed_pc_pub;

void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  tf::StampedTransform registered2sensorTf;
  std::string registered_frame = msg->header.frame_id;
  tf::TransformListener m_tfListener;
  std::cout<<"time="<<msg->header.stamp<<std::endl;
  m_tfListener.waitForTransform(sensor_frame, registered_frame, ros::Time(0), ros::Duration(0.4));
  m_tfListener.lookupTransform(sensor_frame, registered_frame, ros::Time(0), registered2sensorTf);
  Eigen::Matrix4f registered2sensor;
  pcl_ros::transformAsMatrix(registered2sensorTf, registered2sensor);

  pcl::PointCloud<pcl::PointXYZ> pc;

  pcl::fromROSMsg(*msg, pc);

  PointClouds.clear();
  for(pcl::PointCloud<pcl::PointXYZ>::iterator it = pc.begin(), end = pc.end(); it!=end; ++it)
  {
    pcl::PointXYZ point = *it;

    float x = registered2sensor(0,0)*(point.x) + registered2sensor(0,1)*(point.y) + registered2sensor(0,2)*(point.z) + registered2sensor(0,3);
    float y = registered2sensor(1,0)*(point.x) + registered2sensor(1,1)*(point.y) + registered2sensor(1,2)*(point.z) + registered2sensor(1,3);
    float z = registered2sensor(2,0)*(point.x) + registered2sensor(2,1)*(point.y) + registered2sensor(2,2)*(point.z) + registered2sensor(2,3);

    point.x = x;
    point.y = y;
    point.z = z;

    PointClouds.push_back(point);
  }

  PointClouds.header.stamp = pc.header.stamp;
  PointClouds.header.frame_id = sensor_frame;
  PointClouds.is_dense = false;
  pcl::toROSMsg(PointClouds, renamed_pc);
  renamed_pc_pub.publish(renamed_pc);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_transform");
  ros::NodeHandle nh;

  ros::Subscriber pointcoud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2, &pointcloud_callback);
  renamed_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_sensor", 2);
  ros::Rate loop_rate(5);
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
