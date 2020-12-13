#ifndef MINKINDR_CONVERSIONS_KINDR_MSG_H
#define MINKINDR_CONVERSIONS_KINDR_MSG_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <kindr/minimal/quat-transformation.h>

namespace tf {

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
void poseKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    geometry_msgs::Pose* msg);
void poseMsgToKindr(const geometry_msgs::Pose& msg,
                    kindr::minimal::QuatTransformation* kindr);
void poseStampedKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    const ros::Time& time,
                    const std::string& reference_frame,
                    geometry_msgs::PoseStamped* msg);
// Uses current time.
void poseStampedKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    const std::string& reference_frame,
                    geometry_msgs::PoseStamped* msg);

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::Transform.
void transformKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                         geometry_msgs::Transform* msg);
void transformMsgToKindr(const geometry_msgs::Transform& msg,
                         kindr::minimal::QuatTransformation* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToMsg(const kindr::minimal::RotationQuaternion& kindr,
                          geometry_msgs::Quaternion* msg);
void quaternionMsgToKindr(const geometry_msgs::Quaternion& msg,
                          kindr::minimal::RotationQuaternion* kindr);

// Also the Eigen implementation version of this.
void quaternionKindrToMsg(const Eigen::Quaterniond& kindr,
                          geometry_msgs::Quaternion* msg);
void quaternionMsgToKindr(const geometry_msgs::Quaternion& msg,
                          Eigen::Quaterniond* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void pointKindrToMsg(const Eigen::Vector3d& kindr, geometry_msgs::Point* msg);
void pointMsgToKindr(const geometry_msgs::Point& msg, Eigen::Vector3d* kindr);

void vectorKindrToMsg(const Eigen::Vector3d& kindr,
                      geometry_msgs::Vector3* msg);

void vectorMsgToKindr(const geometry_msgs::Vector3& msg,
                      Eigen::Vector3d* kindr);

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_MSG_H
