#ifndef AVS_DETECTION_COMMON_POSE2D_H
#define AVS_DETECTION_COMMON_POSE2D_H

# include <ntk/core.h>
# include <ntk/geometry/similarity_transform.h>
# include <ntk/geometry/affine_transform.h>

namespace ntk
{

  class VisualObjectView;

  class Pose2D : public ntk::XmlSerializable
  {
  public:
    Pose2D(const ntk::AffineTransform& affine_transform);

  public:
    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element);

  public:
    const ntk::AffineTransform& affineTransform() const { return m_affine_transform; }
    void setSimilarityTransform(const ntk::SimilarityTransform& t) { m_similarity_transform = t; }
    const ntk::SimilarityTransform& similarityTransform() const { return m_similarity_transform; }

  private:
    ntk::AffineTransform m_affine_transform;
    ntk::SimilarityTransform m_similarity_transform;
  };

  const NtkDebug& operator<<(const NtkDebug& os, const Pose2D& p);

} // avs

#endif // AVS_DETECTION_COMMON_POSE2D_H
