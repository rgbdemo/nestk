//
// object_match_lowe.cc
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

#include "sift_object_match_lowe.h"
#include <ntk/ntk.h>

namespace ntk
{

  void SiftObjectMatchLowe ::
  fillXmlElement(XMLNode& element) const
  {
    super::fillXmlElement(element);
    setXmlAttribute(element, "pfa", pfa);
    setXmlAttribute(element, "nbvotes", nbvotes);
  }
  
  void SiftObjectMatchLowe ::
  loadFromXmlElement(const XMLNode& element, ntk::XmlContext* context)
  {
    super::loadFromXmlElement(element, context);
    loadFromXmlAttribute(element, "pfa", pfa);
    loadFromXmlAttribute(element, "nbvotes", nbvotes);
  }
  
  const NtkDebug& operator<< (const NtkDebug& os, const SiftObjectMatchLowe& match)
  {
    os << (SiftObjectMatch&)match << " [pfa=" << match.pfa
        << "] [nbvotes=#" << (int)match.nbvotes;
    return os;
  }

} // end of avs
