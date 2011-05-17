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

#ifndef PCL_UTILS_H
#define PCL_UTILS_H

#ifdef NESTK_USE_PCL

#include <ntk/core.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace ntk
{

/*! XYZ + index to find to correspond point in the RGBDImage */
// FIXME: this is hacky. We should define a custom pcl Point!
typedef pcl::PointXYZRGBA PointXYZIndex;

class RGBDImage;
class Pose3D;

void vectorToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud,
                        const std::vector<cv::Point3f>& points,
                        const std::vector<int>& indices = std::vector<int>());

void rgbdImageToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud, const RGBDImage& image);

void rgbdImageToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud,
                           const RGBDImage& image,
                           const Pose3D& pose,
                           int subsampling_factor = 1);

}

#endif // NESTK_USE_PCL

#endif // PCL_UTILS_H
