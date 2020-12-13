#ifndef MINKINDR_CONVERSIONS_KINDR_XML_H
#define MINKINDR_CONVERSIONS_KINDR_XML_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kindr/minimal/quat-transformation.h>
#include <XmlRpcValue.h>

namespace kindr {
namespace minimal {

// Convert from an xml RPC (from ROS Param server) to a
// kindr::minimal::QuatTransformation.
void xmlRpcToKindr(XmlRpc::XmlRpcValue& xml_rpc,
                   kindr::minimal::QuatTransformation* kindr);

}  // namespace minimal
}  // namespace kindr

#endif  // MINKINDR_CONVERSIONS_KINDR_TF_H
