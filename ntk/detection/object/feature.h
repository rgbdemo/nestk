//
// feature.h
//
// Author: Nicolas Burrus <nicolas.burrus@ensta.fr>, (C) 2007
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//

#ifndef FROL_feature_H
# define FROL_feature_H

# include <ntk/ntk.h>
# include <ntk/geometry/affine_transform.h>

namespace ntk
{

  struct FeatureLocation : public ntk::XmlSerializable
  {
    FeatureLocation()
      : p_image(-1,-1,0), p_world(0,0,0), has_depth(false), scale(-1), orientation(-1), laplacian(0)
    {}

    void setDepth(double new_depth) { p_image.z = new_depth; has_depth = true; }

    virtual void fillXmlElement(XMLNode& element) const
    {
      setXmlAttribute(element, "p_image", p_image);
      setXmlAttribute(element, "p_world", p_world);
      setXmlAttribute(element, "has_depth", has_depth);
      setXmlAttribute(element, "scale", scale);
      setXmlAttribute(element, "orientation", orientation);
      setXmlAttribute(element, "laplacian", laplacian);
    }
    
    virtual void loadFromXmlElement(const XMLNode& element)
    {
      loadFromXmlAttribute(element, "p_image", p_image);
      loadFromXmlAttribute(element, "p_world", p_world);
      loadFromXmlAttribute(element, "has_depth", has_depth);
      loadFromXmlAttribute(element, "scale", scale);
      loadFromXmlAttribute(element, "orientation", orientation);
      loadFromXmlAttribute(element, "laplacian", laplacian);
    }
    
    bool valid() const
    {
      return (p_image.x >= 0) && (p_image.y >= 0) && (scale >= 0)
             && ntk::in_range(-M_PI, orientation, M_PI);
    }
    
    void applyTransform(const ntk::AffineTransform& transform);

    // 2d location in image + depth (if has_depth)
    cv::Point3f p_image;
    // 3d location in world coordinates, only defined if has_depth
    cv::Point3f p_world;
    bool has_depth;
    double scale,orientation;
    int laplacian;
  };

  class FeatureDescriptor : public ntk::XmlSerializable
  {
  public:
    enum Type { FloatDescriptor, ByteDescriptor, Undefined };

  public:
    FeatureDescriptor(Type type, int size);
    FeatureDescriptor() : m_type(Undefined) {}
    Type type() const { return m_type; }

    virtual void fillXmlElement(XMLNode& element) const;
    virtual void loadFromXmlElement(const XMLNode& element);

  public:
    std::vector<unsigned char>& byteDescRef() { return m_desc_byte; }
    std::vector<float>& floatDescRef() { return m_desc_float; }
    const std::vector<unsigned char>& byteDesc() const { return m_desc_byte; }
    const std::vector<float>& floatDesc() const { return m_desc_float; }

    bool hasByteDesc() const { return m_desc_byte.size() > 0; }
    bool hasFloatDesc() const { return m_desc_float.size() > 0; }
    int size() const { return hasByteDesc() ? m_desc_byte.size() : m_desc_float.size(); }

  private:
    Type m_type;
    std::vector<unsigned char> m_desc_byte;
    std::vector<float> m_desc_float;
  };
  ntk_ptr_typedefs(FeatureDescriptor);

} // end of avs

const NtkDebug& operator<<(const NtkDebug& os, const ntk::FeatureLocation& loc);

#endif 	    /* !FROL_feature_H */
