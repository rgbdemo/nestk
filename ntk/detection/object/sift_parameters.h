#ifndef FROL_sift_parameters_H
# define FROL_sift_parameters_H

# include <ntk/core.h>

# include "located_feature.h"

namespace ntk
{

  struct SiftHoughParameters
  {
    SiftHoughParameters();
    
    double delta_scale;
    double delta_orientation;
    double delta_location;
  };
  
  struct SiftParameters
  {
    SiftParameters();
    
    std::string sift_database_type;
    double max_dist_ratio;
    double p_good_match;
    SiftHoughParameters hough;
    double pfa_threshold;
    LocatedFeature::FeatureType feature_type;
    double depth_margin;
    double max_reprojection_error_percent;
  };

} // end of avs

#endif // ndef FROL_sift_parameters_H
