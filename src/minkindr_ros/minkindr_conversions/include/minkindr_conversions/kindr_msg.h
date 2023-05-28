#ifndef MINKINDR_CONVERSIONS_KINDR_MSG_H
#define MINKINDR_CONVERSIONS_KINDR_MSG_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <kindr/minimal/quat-transformation.h>

namespace tf {

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
void poseKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    geometry_msgs::msg::Pose* msg);
void poseMsgToKindr(const geometry_msgs::msg::Pose& msg,
                    kindr::minimal::QuatTransformation* kindr);
void poseStampedKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    const builtin_interfaces::msg::Time& time,
                    const std::string& reference_frame,
                    geometry_msgs::msg::PoseStamped* msg);
// Uses current time.
void poseStampedKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    const std::string& reference_frame,
                    geometry_msgs::msg::PoseStamped* msg);

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::msg::Transform.
void transformKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                         geometry_msgs::msg::Transform* msg);
void transformMsgToKindr(const geometry_msgs::msg::Transform& msg,
                         kindr::minimal::QuatTransformation* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToMsg(const kindr::minimal::RotationQuaternion& kindr,
                          geometry_msgs::msg::Quaternion* msg);
void quaternionMsgToKindr(const geometry_msgs::msg::Quaternion& msg,
                          kindr::minimal::RotationQuaternion* kindr);

// Also the Eigen implementation version of this.
void quaternionKindrToMsg(const Eigen::Quaterniond& kindr,
                          geometry_msgs::msg::Quaternion* msg);
void quaternionMsgToKindr(const geometry_msgs::msg::Quaternion& msg,
                          Eigen::Quaterniond* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void pointKindrToMsg(const Eigen::Vector3d& kindr, geometry_msgs::msg::Point* msg);
void pointMsgToKindr(const geometry_msgs::msg::Point& msg, Eigen::Vector3d* kindr);

void vectorKindrToMsg(const Eigen::Vector3d& kindr,
                      geometry_msgs::msg::Vector3* msg);

void vectorMsgToKindr(const geometry_msgs::msg::Vector3& msg,
                      Eigen::Vector3d* kindr);

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_MSG_H
