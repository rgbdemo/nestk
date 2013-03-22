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

#ifndef FROL_sift_object_finder_H
# define FROL_sift_object_finder_H

# include <ntk/ntk.h>
# include "sift_object_match.h"
# include "object_detector.h"

namespace ntk
{

  class LocatedFeature;
  class SiftPointMatch; ntk_ptr_typedefs(SiftPointMatch);

  class SiftObjectDetector : public ObjectDetector
  {
    typedef ObjectDetector super;

    public:
      SiftObjectDetector(const SiftParameters& parameters);
      virtual ~SiftObjectDetector();

    public:
      virtual void setObjectDatabase(const ObjectDatabasePtr& db);

    public:
      virtual unsigned nbObjectMatches() const = 0;
      virtual const SiftObjectMatch& objectMatch(unsigned idx) const = 0;
      virtual SiftObjectMatch& objectMatch(unsigned idx) = 0;

    public:
      virtual void initializeFindObjects();

    public:
      const std::list<SiftPointMatchConstPtr>& pointMatches() const
      { return m_point_matches; }

      const FeatureIndexer& siftDatabase() const
      { return *m_sift_indexer; }

      const LocatedFeature& imageSiftPoint(unsigned index) const
      {
        ntk_assert(index < m_image_sift_points.size(), "Out of bounds.");
        return *m_image_sift_points[index];
      }

    int nbImageSiftPoints() const { return m_image_sift_points.size(); }

    const SiftParameters& siftParameters() const { return m_params; }

    protected:
      SiftParameters m_params;
      FeatureIndexer* m_sift_indexer;
      std::vector<LocatedFeature*> m_image_sift_points;
      std::list<SiftPointMatchConstPtr> m_point_matches;
      LocatedFeature::FeatureType m_feature_type;
  };

} // end of avs

#endif // ndef FROL_sift_object_finder_H
