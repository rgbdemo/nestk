#ifndef FROL_FeatureIndexerKdt_H
# define FROL_FeatureIndexerKdt_H

# include <ntk/ntk.h>
# include "feature_indexer.h"
# include <opencv2/flann/flann.hpp>
# include <QHash>
# include <QSet>

namespace ntk
{

  void prepareFlannQuery(std::vector<float>& query, const LocatedFeature& feature);
  cv::flann::Index* createFlannIndex(cv::Mat1f& flann_features, const std::vector<LocatedFeature*>& features);

  class FeatureIndexerKdt : public SimpleFeatureIndexer
  {
    public:
      FeatureIndexerKdt(const ObjectDatabase& db, LocatedFeature::FeatureType type);
      virtual ~FeatureIndexerKdt();
    
    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element);

      virtual std::string getIndexerName() const { return "kdt"; }

    public:
      virtual MatchResults findMatches(const LocatedFeature& p) const;

    private:
      virtual void rebuild();
      
    private:
      cv::flann::Index* m_flann_index;
      cv::Mat1f m_flann_features;
  };
  ntk_ptr_typedefs(FeatureIndexerKdt);

} // end of avs

#endif // ndef FROL_FeatureIndexerKdt_H
