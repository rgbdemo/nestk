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
