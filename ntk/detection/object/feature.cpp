//
// feature.cc
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

#include "feature.h"
#include <ntk/ntk.h>

namespace ntk
{

  void FeatureLocation ::
  applyTransform(const ntk::AffineTransform& transform)
  {
    cv::Point2f extr (p_image.x+cos(orientation)*scale,
                      p_image.y+sin(orientation)*scale);

    transform.transformPoint(extr.x, extr.y);
    transform.transformPoint(p_image.x, p_image.y);

    double dx = (extr.x-p_image.x);
    double dy = (extr.y-p_image.y);
    double tanx = dy/dx;
    if (dx < 0) tanx *= -1;
    orientation = atan(tanx);
    if (dx < 0) orientation = M_PI - orientation;
    scale = sqrt(dx*dx+dy*dy);
  }

} // end of avs

namespace ntk
{

  FeatureDescriptor :: FeatureDescriptor(Type type, int size)
    : m_type(type)
  {
    if (m_type == FloatDescriptor)
      m_desc_float.resize(size);
    else
      m_desc_byte.resize(size);
  }

  void FeatureDescriptor :: fillXmlElement(XMLNode& element) const
  {
    setXmlAttribute(element, "type", (int)m_type);
    addXmlRawTextDataChild(element, "desc_byte", m_desc_byte);
    addXmlRawTextDataChild(element, "desc_float", m_desc_float);
  }

  void FeatureDescriptor :: loadFromXmlElement(const XMLNode& element)
  {
    int type = Undefined;
    loadFromXmlAttribute(element, "type", type); m_type = (Type) type;
    loadFromXmlRawTextDataChild(element, "desc_byte", m_desc_byte);
    loadFromXmlRawTextDataChild(element, "desc_float", m_desc_float);
  }

} // end of avs

const NtkDebug& operator<<(const NtkDebug& os, const ntk::FeatureLocation& loc)
{
  os << "in image: " << loc.p_image << " in world: " << loc.p_world << " (hasdepth: "
      << loc.has_depth << ")"
      << " scale="  << loc.scale << " orientation=" << loc.orientation;
  return os;
}
