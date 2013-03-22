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
