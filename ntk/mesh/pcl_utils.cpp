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

#include "pcl_utils.h"
#include "pcl_utils.hpp"

namespace ntk
{

template void vectorToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                 const std::vector<cv::Point3f>& points,
                                 const std::vector<int>& indices = std::vector<int>());
template void vectorToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud,
                                 const std::vector<cv::Point3f>& points,
                                 const std::vector<int>& indices = std::vector<int>());

template void rgbdImageToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const RGBDImage& image);
template void rgbdImageToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud, const RGBDImage& image);

template void rgbdImageToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                    const RGBDImage& image,
                                    const Pose3D& pose,
                                    int subsampling_factor = 1);
template void rgbdImageToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud,
                                    const RGBDImage& image,
                                    const Pose3D& pose,
                                    int subsampling_factor = 1);

template void pointCloudToMesh(ntk::Mesh& mesh,
                               const pcl::PointCloud<PointXYZIndex>& cloud);
template void pointCloudToMesh(ntk::Mesh& mesh,
                               const pcl::PointCloud<pcl::PointXYZ>& cloud);

} // ntk
