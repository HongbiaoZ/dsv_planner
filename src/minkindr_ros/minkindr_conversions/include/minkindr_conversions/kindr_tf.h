#ifndef MINKINDR_CONVERSIONS_KINDR_TF_H
#define MINKINDR_CONVERSIONS_KINDR_TF_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <tf/transform_datatypes.h>

namespace tf {

// Convert a kindr::minimal::QuatTransformation to a 6 DoF geometry msgs pose.
void poseKindrToTF(const kindr::minimal::QuatTransformation& kindr,
                   tf::Pose* tf_type);
void poseTFToKindr(const tf::Pose& tf_type,
                   kindr::minimal::QuatTransformation* kindr);

// Convert a kindr::minimal::QuatTransformation to a geometry_msgs::Transform.
void transformKindrToTF(const kindr::minimal::QuatTransformation& kindr,
                        tf::Transform* tf_type);
void transformTFToKindr(const tf::Transform& tf_type,
                        kindr::minimal::QuatTransformation* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void quaternionKindrToTF(const kindr::minimal::RotationQuaternion& kindr,
                         tf::Quaternion* tf_type);
void quaternionTFToKindr(const tf::Quaternion& tf_type,
                         kindr::minimal::RotationQuaternion* kindr);
// Also the Eigen implementation version of this.
void quaternionKindrToTF(const Eigen::Quaterniond& kindr,
                         tf::Quaternion* tf_type);
void quaternionTFToKindr(const tf::Quaternion& tf_type,
                         Eigen::Quaterniond* kindr);

// A wrapper for the relevant functions in eigen_conversions.
void vectorKindrToTF(const Eigen::Vector3d& kindr, tf::Vector3* tf_type);
void vectorTFToKindr(const tf::Vector3& tf_type, Eigen::Vector3d* kindr);

}  // namespace tf

#endif  // MINKINDR_CONVERSIONS_KINDR_TF_H
