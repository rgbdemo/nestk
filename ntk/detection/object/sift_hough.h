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

#ifndef FROL_sift_hough_H
# define FROL_sift_hough_H

# include <ntk/ntk.h>
# include "sift_parameters.h"
# include "feature_indexer.h"

# include <QHash>

namespace ntk
{

  class VisualObject;
  class SiftPointMatch; ntk_ptr_typedefs(SiftPointMatch);

  struct HoughPoint
  {
    HoughPoint() : object_id(), x(-1), y(-1), orientation(0), scale(0) {}
    VisualObjectViewId object_id;
    int x;
    int y;
    unsigned char orientation;
    int scale;
  };

  using ::qHash;
  inline unsigned qHash(const HoughPoint &key)
  {
    unsigned k = (unsigned)(ulong)(key.object_id.object_id);
    k ^= key.object_id.view_id_in_object;
    k ^= key.x;
    k ^= key.y;
    k ^= key.orientation;
    k ^= key.scale;
    return k;
  }

  inline
  bool operator< (const HoughPoint& p1, const HoughPoint& p2)
  {
    if (p1.object_id.view_id_in_database != p2.object_id.view_id_in_database)
      return p1.object_id.view_id_in_database < p2.object_id.view_id_in_database;
    if (p1.x != p2.x) return p1.x < p2.x;
    if (p1.y != p2.y) return p1.y < p2.y;
    if (p1.orientation != p2.orientation) return p1.orientation < p2.orientation;
    if (p1.scale != p2.scale) return p1.scale < p2.scale;
    return false;
  }

  inline
  bool operator==(const HoughPoint& p1, const HoughPoint& p2)
  {
    return (p1.object_id.view_id_in_database == p2.object_id.view_id_in_database)
      && (p1.x == p2.x) && (p1.y == p2.y)
      && (p1.orientation == p2.orientation)
      && (p1.scale == p2.scale);
  }
  
  const NtkDebug& operator <<(const NtkDebug& os, const HoughPoint& p);

  class SiftHough
  {
    public:
      typedef std::map< HoughPoint, std::list<SiftPointMatchConstPtr> > clusters_type;

    public:
      SiftHough(const SiftHoughParameters& parameters)
        : m_params(parameters)
      {}

    public:
      void clear() { m_clusters.clear(); }
      
      const clusters_type& clusters() const
      { return m_clusters; }

      clusters_type& clusters()
      { return m_clusters; }

      bool areNeighbors(const HoughPoint& p1, const HoughPoint& p2) const;

    public:
      void houghpointsFromMatch(std::list<HoughPoint>& points, SiftPointMatchConstPtr match);
      void vote(SiftPointMatchConstPtr match);

    private:
      SiftHoughParameters m_params;
      clusters_type m_clusters;
      mutable RecursiveQMutex m_lock;
  };

} // end of avs

#endif // ndef FROL_sift_hough_H
