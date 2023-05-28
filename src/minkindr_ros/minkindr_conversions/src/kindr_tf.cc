#include "minkindr_conversions/kindr_tf.h"

#include <glog/logging.h>

namespace tf {

void transformKindrToTF(const kindr::minimal::QuatTransformation& kindr,
                        tf2::Transform* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf2::Vector3 origin;
  tf2::Quaternion rotation;
  vectorKindrToTF(kindr.getPosition(), &origin);
  quaternionKindrToTF(kindr.getRotation(), &rotation);
  tf_type->setOrigin(origin);
  tf_type->setRotation(rotation);
}

void transformTFToKindr(const tf2::Transform& tf_type,
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
                         tf2::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf2::Quaternion quat;
  quat.setX(kindr.toImplementation().x());
  quat.setY(kindr.toImplementation().y());
  quat.setZ(kindr.toImplementation().z());
  quat.setW(kindr.toImplementation().w());
  *tf_type = quat;
  // quaternionEigenToTF(kindr.toImplementation(), *tf_type);
}

void quaternionTFToKindr(const tf2::Quaternion& tf_type,
                         kindr::minimal::RotationQuaternion* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond quat;
  quat.x() = tf_type.getX();
  quat.y() = tf_type.getY();
  quat.z() = tf_type.getZ();
  quat.w() = tf_type.getW();
  // quaternionTFToEigen(tf_type, quat);
  *kindr = kindr::minimal::RotationQuaternion(quat);
}

void quaternionKindrToTF(const Eigen::Quaterniond& kindr,
                         tf2::Quaternion* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf2::Quaternion quat;
  quat.setX(kindr.x());
  quat.setY(kindr.y());
  quat.setZ(kindr.z());
  quat.setW(kindr.w());
  *tf_type = quat;
  // quaternionEigenToTF(kindr, *tf_type);
}

void quaternionTFToKindr(const tf2::Quaternion& tf_type,
                         Eigen::Quaterniond* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Quaterniond quat;
  quat.x() = tf_type.getX();
  quat.y() = tf_type.getY();
  quat.z() = tf_type.getZ();
  quat.w() = tf_type.getW();
  *kindr = quat;
  // quaternionTFToEigen(tf_type, *kindr);
}

void vectorKindrToTF(const Eigen::Vector3d& kindr, tf2::Vector3* tf_type) {
  CHECK_NOTNULL(tf_type);
  tf2::Vector3 vec;
  vec.setX(kindr.x());
  vec.setY(kindr.y());
  vec.setZ(kindr.z());
  *tf_type = vec;
  // vectorEigenToTF(kindr, *tf_type);
}

void vectorTFToKindr(const tf2::Vector3& tf_type, Eigen::Vector3d* kindr) {
  CHECK_NOTNULL(kindr);
  Eigen::Vector3d vec(tf_type.getX(), tf_type.getY(), tf_type.getZ());
  *kindr = vec;
  // vectorTFToEigen(tf_type, *kindr);
}

}  // namespace tf
