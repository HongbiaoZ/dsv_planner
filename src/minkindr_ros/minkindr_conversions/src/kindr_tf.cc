#include "minkindr_conversions/kindr_tf.h"

#include <tf_conversions/tf_eigen.h>
#include <glog/logging.h>

namespace tf {

void poseKindrToTF(const kindr::minimal::QuatTransformation& kindr,
                   tf::Pose* tf_type) {
  transformKindrToTF(kindr, tf_type);
}

void poseTFToKindr(const tf::Pose& tf_type,
                   kindr::minimal::QuatTransformation* kindr) {
  transformTFToKindr(tf_type, kindr);
}

void transformKindrToTF(const kindr::minimal::QuatTransformation& kindr,
                        tf::Transform* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf::Vector3 origin;
  tf::Quaternion rotation;
  vectorKindrToTF(kindr.getPosition(), &origin);
  quaternionKindrToTF(kindr.getRotation(), &rotation);
  tf_type->setOrigin(origin);
  tf_type->setRotation(rotation);
}

void transformTFToKindr(const tf::Transform& tf_type,
                        kindr::minimal::QuatTransformation* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;

  quaternionTFToKindr(tf_type.getRotation(), &rotation);
  vectorTFToKindr(tf_type.getOrigin(), &position);

  // Enforce positive w.
  if (rotation.w() < 0) {
    rotation.coeffs() = -rotation.coeffs();
  }

  *kindr = kindr::minimal::QuatTransformation(rotation, position);
}

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToTF(const kindr::minimal::RotationQuaternion& kindr,
                         tf::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  quaternionEigenToTF(kindr.toImplementation(), *tf_type);
}

void quaternionTFToKindr(const tf::Quaternion& tf_type,
                         kindr::minimal::RotationQuaternion* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond quat;
  quaternionTFToEigen(tf_type, quat);
  *kindr = kindr::minimal::RotationQuaternion(quat);
}

void quaternionKindrToTF(const Eigen::Quaterniond& kindr,
                         tf::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  quaternionEigenToTF(kindr, *tf_type);
}

void quaternionTFToKindr(const tf::Quaternion& tf_type,
                         Eigen::Quaterniond* kindr) {
  CHECK_NOTNULL(kindr);
  quaternionTFToEigen(tf_type, *kindr);
}

void vectorKindrToTF(const Eigen::Vector3d& kindr, tf::Vector3* tf_type) {
  CHECK_NOTNULL(tf_type);
  vectorEigenToTF(kindr, *tf_type);
}

void vectorTFToKindr(const tf::Vector3& tf_type, Eigen::Vector3d* kindr) {
  CHECK_NOTNULL(kindr);
  vectorTFToEigen(tf_type, *kindr);
}

}  // namespace tf
