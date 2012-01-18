//
// object_match.h
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

#ifndef FROL_sift_object_match_H
# define FROL_sift_object_match_H

# include <ntk/ntk.h>
# include "sift_hough.h"
# include "object_match.h"
# include "sift_point_match.h"

namespace ntk
{

  class SiftPointMatch; ntk_ptr_typedefs(SiftPointMatch);
  class VisualObject;
  class ObjectDetectorData;

  class SiftObjectMatch : public ObjectMatch
  {
    typedef ObjectMatch super;

    public:
      SiftObjectMatch(const ObjectDetectorData& data,
                      const ObjectPosePtr& pose)
        : ObjectMatch(data, pose)
      {}

      SiftObjectMatch() : ObjectMatch()
      {}

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      using ntk::XmlSerializable::loadFromXmlElement;
      virtual void loadFromXmlElement(const XMLNode& element, ntk::XmlContext* context);
      virtual void debug(const NtkDebug& os) const;
      int nbMatchesWithDepth() const;

    public:
      SiftPointMatchConstPtrSet point_matches;
      HoughPoint hough_point;
  };
  ntk_ptr_typedefs(SiftObjectMatch);

  inline const NtkDebug& operator<< (const NtkDebug& os, const SiftObjectMatch& match)
  { match.debug(os); return os; }

  struct SiftObjectMatchVoteLt
  {
    bool operator()(SiftObjectMatchConstPtr m1,
                    SiftObjectMatchConstPtr m2) const
    { return m1->score() < m2->score(); }
  };

} // end of avs

#endif // ndef FROL_sift_object_match_H
