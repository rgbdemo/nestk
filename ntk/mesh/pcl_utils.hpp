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

#ifndef NTK_MESH_PCL_UTILS_HPP
#define NTK_MESH_PCL_UTILS_HPP

#include "pcl_utils.h"

namespace ntk
{

template <class PointT>
void vectorToPointCloud(pcl::PointCloud<PointT>& cloud,
                        const std::vector<cv::Point3f>& points,
                        const std::vector<int>& indices = std::vector<int>())
{
    cloud.width  = points.size();
    cloud.height = 1;
    cloud.points.resize (cloud.width * cloud.height);

    bool has_indices = (indices.size() == points.size());

    foreach_idx(i, points)
    {
        const cv::Point3f& p = points[i];
        cloud.points[i].x = p.x;
        cloud.points[i].y = p.y;
        cloud.points[i].z = p.z;
        // FIXME: static if
#if 0
        if (has_indices)
            cloud.points[i].rgba = indices[i];
#endif
    }
}

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud, const RGBDImage& image)
{
    if (!image.calibration())
        ntk_throw_exception("No calibration data in image.");

    rgbdImageToPointCloud(cloud, image, *image.calibration()->depth_pose);
}

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud,
                           const RGBDImage& image,
                           const Pose3D& pose,
                           int subsampling_factor)
{
    std::vector<cv::Point3f> points;
    std::vector<int> indices;

    for (int r = 0; r < image.depth().rows; r += subsampling_factor)
        for (int c = 0; c < image.depth().cols; c += subsampling_factor)
        {
            float d = image.depth()(r,c);
            bool mask_ok = !image.depthMask().data || image.depthMask()(r,c);
            if (d < 1e-5 || !mask_ok)
                continue;
            cv::Point3f p = pose.unprojectFromImage(cv::Point2f(c,r),d);
            points.push_back(p);
            indices.push_back(r*image.depth().cols+c);
        }

    vectorToPointCloud(cloud, points, indices);
}

template <class PointT>
void pointCloudToMesh(ntk::Mesh& mesh,
                      const pcl::PointCloud<PointT>& cloud)
{
    mesh.clear();
    mesh.vertices.resize(cloud.size());
    foreach_idx(i, cloud.points)
    {
        mesh.vertices[i] = cv::Point3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    }
}

} // ntk

#endif // NTK_MESH_PCL_UTILS_HPP
