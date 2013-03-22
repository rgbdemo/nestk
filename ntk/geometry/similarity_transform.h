#ifndef NTK_GEOMETRY_SIMILARITYTRANSFORM_H
#define NTK_GEOMETRY_SIMILARITYTRANSFORM_H

#include <ntk/core.h>
#include <ntk/utils/xml_serializable.h>

namespace ntk
{

class Polygon2d;

class SimilarityTransform : public ntk::XmlSerializable
{
public:
  SimilarityTransform() : m_translation(0,0), m_rotation(0), m_scale(1)
  {
    m_matrix.create(3,3);
    setIdentity(m_matrix);
  }

  SimilarityTransform(const cv::Vec2f& translation, double rotation, double scale)
    : m_translation(translation), m_rotation(rotation), m_scale(scale)
  {
    computeMatrix();
  }

public:
  virtual void fillXmlElement(XMLNode& element) const;
  virtual void loadFromXmlElement(const XMLNode& element);

public:
  const cv::Vec2f& translation() const { return m_translation; }
  double rotation() const { return m_rotation; }
  double scale() const { return m_scale; }

  SimilarityTransform& applyTransformBefore(const cv::Vec2f& translation, double rotation, double scale);
  SimilarityTransform& applyTransformAfter(const cv::Vec2f& translation, double rotation, double scale);

public:
  cv::Point2f transform(const cv::Point2f& p) const;
  void transform(const cv::Rect_<float>& box, Polygon2d& polygon) const;
  cv::Vec2f transformRotationOnly(const cv::Vec2f& p) const;

protected:
  void computeMatrix();

private:
  cv::Mat1f m_matrix;
  cv::Vec2f m_translation;
  double m_rotation;
  double m_scale;
};

}

#endif // NTK_GEOMETRY_SIMILARITYTRANSFORM_H
