#include "pose_2d.h"
#include "object_database.h"

using namespace ntk;

namespace ntk
{

  Pose2D :: Pose2D(const ntk::AffineTransform& affine_transform)
    : m_affine_transform(affine_transform)
  {
  }

  void Pose2D ::
  fillXmlElement(XMLNode& element) const
  {
    addXmlChild(element, "affine_transform", *AffineTransformXmlSerializer::get(m_affine_transform));
  }

  void Pose2D ::
  loadFromXmlElement(const XMLNode& element)
  {
    loadFromXmlChild(element, "affine_transform", *AffineTransformXmlSerializer::get(m_affine_transform));
  }

  const NtkDebug& operator<<(const NtkDebug& os, const Pose2D& pose)
  {
    os << "tx=" << pose.affineTransform().mat02
       << " ty=" << pose.affineTransform().mat12;
    return os;
  }

}
