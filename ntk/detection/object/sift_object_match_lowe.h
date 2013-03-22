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

#ifndef FROL_sift_object_match_lowe_H
# define FROL_sift_object_match_lowe_H

# include "sift_object_match.h"

namespace ntk
{

  class SiftObjectMatchLowe : public SiftObjectMatch
  {
    typedef SiftObjectMatch super;

    public:
      SiftObjectMatchLowe(const ObjectDetectorData& data,
                          const ObjectPosePtr& pose)
        : SiftObjectMatch(data, pose)
      {}

      SiftObjectMatchLowe()
        : SiftObjectMatch(), nbvotes(0), pfa(0)
      {}

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element, ntk::XmlContext* context);

      virtual double score() const { return -pfa; }

    public:
      unsigned nbvotes;
      double pfa;
  };
  ntk_ptr_typedefs(SiftObjectMatchLowe);

  const NtkDebug& operator<< (const NtkDebug& os, const SiftObjectMatchLowe& match);

} // end of avs

#endif // ndef FROL_sift_object_match_lowe_H
