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

#include "sift_hough.h"
#include "sift_point_match.h"
#include "object_database.h"
#include "object_pose.h"
#include "visual_object.h"
#include <ntk/ntk.h>

using namespace ntk;

namespace ntk
{

  /************************************************************************/

  const NtkDebug& operator <<(const NtkDebug& os, const HoughPoint& p)
  {
    if (p.object_id.isValid())
      os << "[" << p.object_id.object_id << " " << p.object_id.view_id_in_object
         << " " << p.x << " " << p.y << " " << p.orientation << " " << p.scale << "]";
    return os;
  }

  /************************************************************************/

  void SiftHough :: houghpointsFromMatch(std::list<HoughPoint>& points, SiftPointMatchConstPtr match)
  {
    ntk_assert(match, "Null pointer.");
    const SiftPointTransform& point_transform = match->pointTransform();

    std::vector<unsigned char> orientation_bins(2);
    std::vector<int> scale_bins(2);
    std::vector<int> x_bins(2);
    std::vector<int> y_bins(2);

    // orientation bins
    {
      unsigned max_bins = (unsigned) (std::floor((2.0*M_PI)/m_params.delta_orientation)-1);
      double o = point_transform.orientation / m_params.delta_orientation;
      orientation_bins[0] = (unsigned) std::floor(o);
      if ((o-orientation_bins[0]) < 0.5)
      {
        if (orientation_bins[0] == 0) orientation_bins[1] = max_bins;
        else orientation_bins[1] = orientation_bins[0] - 1;
      }
      else
      {
        if (orientation_bins[0] == max_bins) orientation_bins[1] = 0;
        else orientation_bins[1] = orientation_bins[0] + 1;
      }
      ntk_dbg_print(point_transform.orientation, 2);
      ntk_dbg_print(o, 2);
      ntk_dbg_print(orientation_bins[0], 2);
      ntk_dbg_print(orientation_bins[1], 2);
    }

    // scale bins
    {
      double s = log(point_transform.scale)/log(2.0);
      scale_bins[0] = (int)floor(s);
      int s2;
      if (s > scale_bins[0]+0.5) s2 = scale_bins[0] + 1;
      else s2 = scale_bins[0] - 1;
      scale_bins[1] = s2;
    }

    const VisualObjectView& model = match->pose()->modelView();
    cv::Rect_<float> model_bbox = model.boundingRect();
    const ObjectPose& model_pose = *match->pose();

    // Vote for 16 bins in most cases.
    for (unsigned s = 0; s < scale_bins.size(); ++s)
    for (unsigned o = 0; o < orientation_bins.size(); ++o)
    {
      HoughPoint p;
      p.object_id = model.id();
      p.orientation = orientation_bins[o];
      p.scale = scale_bins[s];

      double actual_scale = pow(2.0, p.scale);
      ntk_dbg_print(actual_scale, 2);
      // double max_dim = (model_bbox.width()*actual_scale + model_bbox.height()*actual_scale) / 2.0;
      double max_dim = std::max(model_bbox.width*actual_scale, model_bbox.height*actual_scale);
      max_dim *= m_params.delta_location;
      double x = model_pose.projectedCenter().x / max_dim;
      double y = model_pose.projectedCenter().y / max_dim;
      x_bins[0] = (int)floor(x);
      x_bins[1] = x_bins[0] + ((x-x_bins[0]) < 0.5 ? -1 : 1);
      y_bins[0] = (int)floor(y);
      y_bins[1] = y_bins[0] + ((y-y_bins[0]) < 0.5 ? -1 : 1);
      for (unsigned y = 0; y < y_bins.size(); ++y)
      for (unsigned x = 0; x < x_bins.size(); ++x)
      {
        p.x = x_bins[x];
        p.y = y_bins[y];
        points.push_back(p);
      }
    }
  }

  void SiftHough :: vote(SiftPointMatchConstPtr match)
  {
    std::list<HoughPoint> points;
    houghpointsFromMatch(points, match);
    QMutexLocker locker(&m_lock);
    foreach_const_it(it, points, std::list<HoughPoint>)
      m_clusters[*it].push_back(match);
  }

  bool SiftHough :: areNeighbors(const HoughPoint& p1, const HoughPoint& p2) const
  {
    // if (p1.object != p2.object) return false;
    if (std::abs(p1.x - p2.x) > 2) return false;
    if (std::abs(p1.y - p2.y) > 2) return false;
    if (std::abs(p1.scale - p2.scale) > 1) return false;
    return true;
    unsigned max_bins = (unsigned) (std::floor((2.0*M_PI)/m_params.delta_orientation)-1);
    if (p1.orientation == 0)
    {
      if (p2.orientation > 1 && p2.orientation < max_bins) return false;
    }
    else if (p2.orientation == 0)
    {
      if (p1.orientation > 1 && p1.orientation < max_bins) return false;
    }
    else if (std::abs((int)p1.orientation - (int)p2.orientation) > 1)
      return false;
    return true;
  }

  /************************************************************************/

} // end of avs
