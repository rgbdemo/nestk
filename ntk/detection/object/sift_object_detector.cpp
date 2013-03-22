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

#include "sift_object_detector.h"
#include "located_feature.h"
#include "feature_indexer_kdt.h"
#include "object_database.h"
#include <ntk/ntk.h>

using namespace cv;
using namespace ntk;

namespace ntk
{

  SiftObjectDetector :: SiftObjectDetector(const SiftParameters& params)
    : m_params(params), m_sift_indexer(0)
  {

  }

  void SiftObjectDetector :: setObjectDatabase(const ObjectDatabasePtr& db)
  {
    super::setObjectDatabase(db);

    if (m_sift_indexer) delete m_sift_indexer;
    if (!db) return;

    if (m_params.sift_database_type == "simple")
      m_sift_indexer = new SimpleFeatureIndexer(objectDatabase(), m_params.feature_type);
#ifdef NESTK_DISABLED_ADS_NOT_YET_IMPORTED
    else if (m_params.sift_database_type == "lsh")
      m_sift_indexer = new FeatureIndexerLSH(objectDatabase(), m_params.feature_type);
#endif
    else if (m_params.sift_database_type == "kdt")
      m_sift_indexer = new FeatureIndexerKdt(objectDatabase(), m_params.feature_type);
#ifdef NESTK_DISABLED_ADS_NOT_YET_IMPORTED
    else if (m_params.sift_database_type == "with_clusters")
      m_sift_indexer = new FeatureIndexerWithClusters(objectDatabase(), m_params.feature_type);
    else if (m_params.sift_database_type == "with_clusters_no_mean")
      m_sift_indexer = new FeatureIndexerWithClustersNoMean(objectDatabase(), m_params.feature_type);
#endif
    else
      ntk_throw_exception("Invalid sift database type.");
    TimeCount pc ("loading sift indexer");
    m_sift_indexer->loadOrBuild();
    pc.stop();
  }

  void SiftObjectDetector :: initializeFindObjects()
  {
    super::initializeFindObjects();

    m_point_matches.clear();
    m_image_sift_points.clear();

    std::list<LocatedFeature*> points;

    ntk::TimeCount tc_init("initialize", 1);
    compute_feature_points(points,
                           m_data.image,
                           siftDatabase().featureType());
    tc_init.elapsedMsecs(" compute feature points: ");

    std::copy(stl_bounds(points), std::back_inserter(m_image_sift_points));
    ntk_dbg_print(m_image_sift_points.size(), 1);
    tc_init.stop();
  }

  SiftObjectDetector :: ~SiftObjectDetector()
  {
    if (m_sift_indexer) delete m_sift_indexer;
    foreach_it(it, m_image_sift_points, std::vector<LocatedFeature*>)
      delete *it;
  }

} // end of avs
