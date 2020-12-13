#include "minkindr_conversions/kindr_xml.h"

#include <glog/logging.h>

namespace kindr {
namespace minimal {

void xmlRpcToKindr(XmlRpc::XmlRpcValue& xml_rpc,
                   kindr::minimal::QuatTransformation* kindr) {
  kindr::minimal::QuatTransformation::RotationMatrix temp_rot_matrix;
  kindr::minimal::QuatTransformation::Position temp_translation;

  if (kindr == nullptr) {
    LOG(ERROR) << "Null pointer given";
    return;
  }
  if (xml_rpc.size() != 4) {
    LOG(ERROR) << "XmlRpc matrix has " << xml_rpc.size() << " rows";
    return;
  }
  // read raw inputs
  for (size_t i = 0; i < 3; ++i) {
    if (xml_rpc[i].size() != 4) {
      LOG(ERROR) << "XmlRpc matrix has " << xml_rpc[i].size()
                 << " columns in its " << i << " row";
      return;
    }
    for (size_t j = 0; j < 3; ++j) {
      temp_rot_matrix(i, j) = xml_rpc[i][j];
    }
    temp_translation(i) = xml_rpc[i][3];
  }

  // renormalize rotation to correct for rounding error when yaml was written
  kindr::minimal::RotationQuaternion temp_rot_quat =
      kindr::minimal::RotationQuaternion::constructAndRenormalize(
          temp_rot_matrix);

  // recombine
  *kindr = kindr::minimal::QuatTransformation(temp_rot_quat, temp_translation);
}

}  // namespace minimal
}  // namespace kindr
