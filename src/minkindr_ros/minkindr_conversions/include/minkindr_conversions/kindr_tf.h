#ifndef MINKINDR_CONVERSIONS_KINDR_TF_H
#define MINKINDR_CONVERSIONS_KINDR_TF_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace tf {

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::msg::Transform.
void transformKindrToTF(const kindr::minimal::QuatTransformation& kindr,
                        tf2::Transform* tf_type);
void transformTFToKindr(const tf2::Transform& tf_type,
                        kindr::minimal::QuatTransformation* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToTF(const kindr::minimal::RotationQuaternion& kindr,
                         tf2::Quaternion* tf_type);
void quaternionTFToKindr(const tf2::Quaternion& tf_type,
                         kindr::minimal::RotationQuaternion* kindr);
// Also the Eigen implementation version of this.
void quaternionKindrToTF(const Eigen::Quaterniond& kindr,
                         tf2::Quaternion* tf_type);
void quaternionTFToKindr(const tf2::Quaternion& tf_type,
                         Eigen::Quaterniond* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void vectorKindrToTF(const Eigen::Vector3d& kindr, tf2::Vector3* tf_type);
void vectorTFToKindr(const tf2::Vector3& tf_type, Eigen::Vector3d* kindr);

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_TF_H
