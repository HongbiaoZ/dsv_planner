#include "minkindr_conversions/kindr_msg.h"

#include <eigen_conversions/eigen_msg.h>
#include <glog/logging.h>

namespace tf {

void poseKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    geometry_msgs::Pose* msg) {
  CHECK_NOTNULL(msg);
  pointKindrToMsg(kindr.getPosition(), &msg->position);
  quaternionKindrToMsg(kindr.getRotation(), &msg->orientation);
}

void poseMsgToKindr(const geometry_msgs::Pose& msg,
                    kindr::minimal::QuatTransformation* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;

  quaternionMsgToKindr(msg.orientation, &rotation);
  pointMsgToKindr(msg.position, &position);

  *kindr = kindr::minimal::QuatTransformation(rotation, position);
}

void poseStampedKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    const ros::Time& time,
                    const std::string& reference_frame,
                    geometry_msgs::PoseStamped* msg) {
  CHECK_NOTNULL(msg);
  msg->header.frame_id = reference_frame;
  msg->header.stamp = time;
  poseKindrToMsg(kindr, &msg->pose);
}

void poseStampedKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                    const std::string& reference_frame,
                    geometry_msgs::PoseStamped* msg) {
  poseStampedKindrToMsg(kindr, ros::Time(), reference_frame, msg);
}

void transformKindrToMsg(const kindr::minimal::QuatTransformation& kindr,
                         geometry_msgs::Transform* msg) {
  CHECK_NOTNULL(msg);
  vectorKindrToMsg(kindr.getPosition(), &msg->translation);
  quaternionKindrToMsg(kindr.getRotation(), &msg->rotation);
}

void transformMsgToKindr(const geometry_msgs::Transform& msg,
                         kindr::minimal::QuatTransformation* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;

  quaternionMsgToKindr(msg.rotation, &rotation);
  vectorMsgToKindr(msg.translation, &position);

  *kindr = kindr::minimal::QuatTransformation(rotation, position);
}

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToMsg(const kindr::minimal::RotationQuaternion& kindr,
                          geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr.toImplementation(), *msg);
}

void quaternionMsgToKindr(const geometry_msgs::Quaternion& msg,
                          kindr::minimal::RotationQuaternion* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond quat;
  quaternionMsgToEigen(msg, quat);
  *kindr = kindr::minimal::RotationQuaternion(quat);
}

void quaternionKindrToMsg(const Eigen::Quaterniond& kindr,
                          geometry_msgs::Quaternion* msg) {
  CHECK_NOTNULL(msg);
  quaternionEigenToMsg(kindr, *msg);
}

void quaternionMsgToKindr(const geometry_msgs::Quaternion& msg,
                          Eigen::Quaterniond* kindr) {
  CHECK_NOTNULL(kindr);
  quaternionMsgToEigen(msg, *kindr);
}

void pointKindrToMsg(const Eigen::Vector3d& kindr, geometry_msgs::Point* msg) {
  CHECK_NOTNULL(msg);
  pointEigenToMsg(kindr, *msg);
}

void pointMsgToKindr(const geometry_msgs::Point& msg, Eigen::Vector3d* kindr) {
  CHECK_NOTNULL(kindr);
  pointMsgToEigen(msg, *kindr);
}

void vectorKindrToMsg(const Eigen::Vector3d& kindr,
                      geometry_msgs::Vector3* msg) {
  CHECK_NOTNULL(msg);
  vectorEigenToMsg(kindr, *msg);
}

void vectorMsgToKindr(const geometry_msgs::Vector3& msg,
                      Eigen::Vector3d* kindr) {
  CHECK_NOTNULL(kindr);
  vectorMsgToEigen(msg, *kindr);
}

}  // namespace tf
