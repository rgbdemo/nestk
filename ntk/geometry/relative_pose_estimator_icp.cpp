/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef NESTK_USE_PCL
# error NESTK_USE_PCL should be defined!
# define NESTK_USE_PCL
#endif

#include "relative_pose_estimator_icp.hpp"

#include <pcl/point_types.h>

template class ntk::RelativePoseEstimatorICP<pcl::PointXYZ>;
template class ntk::RelativePoseEstimatorICP<pcl::PointNormal>;
template class ntk::RelativePoseEstimatorICPWithNormals<pcl::PointNormal>;
template class ntk::RelativePoseEstimatorGICP<pcl::PointNormal>;
