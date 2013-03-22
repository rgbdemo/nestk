
#ifndef NESTK_USE_PCL
# error NESTK_USE_PCL should be defined!
# define NESTK_USE_PCL
#endif

#include "relative_pose_estimator_icp.hpp"

#include <pcl/point_types.h>

template class ntk::RelativePoseEstimatorICP<pcl::PointXYZ>;
template class ntk::RelativePoseEstimatorICP<pcl::PointNormal>;
template class ntk::RelativePoseEstimatorICPWithNormals<pcl::PointNormal>;
// template class ntk::RelativePoseEstimatorGICP<pcl::PointNormal>;
