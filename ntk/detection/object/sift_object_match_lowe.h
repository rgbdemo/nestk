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
