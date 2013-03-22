/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

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
