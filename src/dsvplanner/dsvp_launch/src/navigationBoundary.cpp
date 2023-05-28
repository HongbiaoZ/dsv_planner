#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

const double PI = 3.1415926;

string boundary_file_dir;
bool sendBoundary = true;
int sendBoundaryInterval = 2;
int sendBoundaryCount = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

// reading boundary from file function
void readBoundaryFile()
{
  FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(boundary_file, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(boundary_file, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  boundary->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(boundary_file, "%f", &point.x);
    val2 = fscanf(boundary_file, "%f", &point.y);
    val3 = fscanf(boundary_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    point.z = 0;
    boundary->push_back(point);
  }

  if (boundary->points[0].x != boundary->points[pointNum - 1].x || boundary->points[0].y != boundary->points[pointNum - 1].y) {
    boundary->push_back(boundary->points[0]);
  }

  fclose(boundary_file);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("navigationBoundary");

  nh->get_parameter("boundary_file_dir", boundary_file_dir);
  nh->get_parameter("sendBoundary", sendBoundary);
  nh->get_parameter("sendBoundaryInterval", sendBoundaryInterval);

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pubBoundary = nh->create_publisher<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5);
  geometry_msgs::msg::PolygonStamped boundaryMsgs;
  boundaryMsgs.header.frame_id = "map";

  // read boundary from file
  if (sendBoundary) {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++) {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  rclcpp::WallRate loopRate(100);
  while (rclcpp::ok()) {
    // publish boundary messages at certain frame rate
    sendBoundaryCount++;
    if (sendBoundaryCount >= 100 * sendBoundaryInterval && sendBoundary) {
      pubBoundary->publish(boundaryMsgs);
      sendBoundaryCount = 0;
    }
    loopRate.sleep();
  }

  return 0;
}
