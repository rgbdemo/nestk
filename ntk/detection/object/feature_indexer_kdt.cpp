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

#include "feature_indexer_kdt.h"
#include "object_database.h"
#include "visual_object.h"
#include "located_feature.h"

using namespace ntk;

namespace ntk
{
  FeatureIndexerKdt :: FeatureIndexerKdt(const ObjectDatabase& db,
                                         LocatedFeature::FeatureType type)
      : SimpleFeatureIndexer(db,type),
        m_flann_index(0)
  {
  }

  FeatureIndexerKdt :: ~FeatureIndexerKdt()
  {
    delete m_flann_index;
  }

  void FeatureIndexerKdt :: rebuild()
  {
    SimpleFeatureIndexer::rebuild();
    m_flann_index = createFlannIndex(m_flann_features, m_points);
  }

  void FeatureIndexerKdt :: fillXmlElement(XMLNode& element) const
  {
    SimpleFeatureIndexer::fillXmlElement(element);
  }

  void FeatureIndexerKdt :: loadFromXmlElement(const XMLNode& element)
  {
    SimpleFeatureIndexer :: loadFromXmlElement(element);
    m_flann_index = createFlannIndex(m_flann_features, m_points);
  }

  cv::flann::Index* createFlannIndex(cv::Mat1f& flann_features, const std::vector<LocatedFeature*>& features)
  {
    flann_features = cv::Mat1f(features.size(), features[0]->descriptorSize());
    for (int r = 0; r < features.size(); ++r)
    {
      const FeatureDescriptor& desc = features[r]->descriptor();
      for (int c = 0; c < flann_features.cols; ++c)
        flann_features(r,c) = desc.hasByteDesc() ? desc.byteDesc()[c] : desc.floatDesc()[c];
    }
    return new cv::flann::Index(flann_features, cv::flann::KDTreeIndexParams(4));
  }

  void prepareFlannQuery(std::vector<float>& query, const LocatedFeature& feature)
  {
    query.resize(feature.descriptorSize());
    const FeatureDescriptor& desc = feature.descriptor();
    if (desc.hasByteDesc())
      std::copy(desc.byteDesc().begin(), desc.byteDesc().end(), query.begin());
    else
      query = desc.floatDesc();
  }

  FeatureIndexerKdt::MatchResults
  FeatureIndexerKdt :: findMatches(const LocatedFeature& p) const
  {
    float min_dist = std::numeric_limits<float>::max();
    float min_dist2 = std::numeric_limits<float>::max();

    const LocatedFeature* closest_id = 0;
    const LocatedFeature* closest_id2 = 0;

    std::vector<float> query;
    prepareFlannQuery(query, p);
    std::vector<int> indices(2, -1);
    std::vector<float> dists(2, 0);
    {
      QWriteLocker locker(&m_lock);
      m_flann_index->knnSearch(query, indices, dists, 2, cv::flann::SearchParams(64));
    }

    MatchResults c;
    if (indices[0] >= 0 && indices[1] >= 0)
    {
      const LocatedFeature* p1 = m_points[indices[0]];
      const LocatedFeature* p2 = m_points[indices[1]];

      // No muy eficaz
      if (0
          && p.location().has_depth
          && p1->location().has_depth)
      {
        double scale_ratio = (p.location().scale / p1->location().scale);
        double z_ratio = (p.location().p_image.z / p1->location().p_image.z);

        double diff = (scale_ratio * z_ratio);
        if (diff < 1) diff = 1.0 / diff;
        if (diff > 1.7) // scale does not match
        {
          // ++nb_filtered;
          if ((dists[0]/dists[1]) < 0.5)
          {
            ntk_dbg_print(diff, 1);
            ntk_dbg_print(scale_ratio, 1);
            ntk_dbg_print(z_ratio, 1);
            ntk_dbg_print(p.location().p_image.z, 1);
            ntk_dbg_print(p1->location().p_image.z, 1);
          }
          return c;
        }
      }

      dists[1] += 1; // avoid degenerate case where dist is 0.
      c.matches.push_back(Match(p1, dists[0]/dists[1]));
    }
    return c;
  }

} // end of avs
