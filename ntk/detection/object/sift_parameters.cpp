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

#include "sift_parameters.h"
#include <ntk/ntk.h>

namespace ntk
{

  SiftHoughParameters :: SiftHoughParameters()
  {
#if 0
    delta_scale = 2;
    delta_orientation = (30.*M_PI)/180.;
    delta_location = 0.2;
#endif

    delta_scale = 2;
    delta_orientation = (40.*M_PI)/180.;
    delta_location = 0.3;
  }

  SiftParameters :: SiftParameters()
  {
    // FIXME: just trying. sift_database_type = "simple";
    sift_database_type = "kdt";
    // sift_database_type = "lsh";
    // sift_database_type = "with_clusters";
    feature_type = LocatedFeature::Feature_FAST;
    // feature_type = LocatedFeature::Feature_SURF64;
    /*max_dist_ratio = 1;
    p_good_match = 0.000001;
    pfa_threshold = 0;*/
    max_dist_ratio = 0.80;
    // max_dist_ratio = 0.99;
    p_good_match = 0.001;
    // pfa_threshold = 0.98;
    // pfa_threshold = 0.999342;
    pfa_threshold = 0.98;
    depth_margin = 0.2;
    max_reprojection_error_percent = 0.1;
  }

} // end of avs
