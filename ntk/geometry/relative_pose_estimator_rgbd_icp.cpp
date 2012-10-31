#include "relative_pose_estimator_rgbd_icp.h"
#include "relative_pose_estimator_rgbd_icp.hpp"

#include <pcl/point_types.h>

namespace ntk
{

template class RelativePoseEstimatorRGBDICP<pcl::PointNormal>;

} // ntk
