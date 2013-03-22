
#include "transformation_estimation_rgbd.h"
#include "transformation_estimation_rgbd.hpp"

#ifdef HAVE_PCL_GREATER_THAN_1_6_0

namespace ntk
{

// template
// class TransformationEstimationRGBD<pcl::PointXYZ, pcl::PointXYZ, float>;

template
class TransformationEstimationRGBD<pcl::PointNormal, pcl::PointNormal, float>;

}

#endif
