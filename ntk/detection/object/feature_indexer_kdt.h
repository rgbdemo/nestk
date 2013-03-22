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
