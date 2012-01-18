//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
