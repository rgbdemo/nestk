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

#include <pcl/ros/conversions.h>

namespace ntk
{

template void vectorToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                 const std::vector<cv::Point3f>& points,
                                 const std::vector<int>& indices);
template void vectorToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud,
                                 const std::vector<cv::Point3f>& points,
                                 const std::vector<int>& indices);

template void rgbdImageToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, const RGBDImage& image, bool keep_dense);
template void rgbdImageToPointCloud(pcl::PointCloud<pcl::PointNormal>& cloud, const RGBDImage& image, bool keep_dense);
template void rgbdImageToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud, const RGBDImage& image, bool keep_dense);

template void rgbdImageToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                    const RGBDImage& image,
                                    const Pose3D& pose,
                                    int subsampling_factor,
                                    bool keep_dense);
template void rgbdImageToPointCloud(pcl::PointCloud<PointXYZIndex>& cloud,
                                    const RGBDImage& image,
                                    const Pose3D& pose,
                                    int subsampling_factor,
                                    bool keep_dense);

template void pointCloudToMesh(ntk::Mesh& mesh,
                               const pcl::PointCloud<PointXYZIndex>& cloud);
template void pointCloudToMesh(ntk::Mesh& mesh,
                               const pcl::PointCloud<pcl::PointXYZ>& cloud);

template
void sampledRgbdImageToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                                  const RGBDImage& image,
                                  const Pose3D& pose,
                                  int n_samples);

void polygonMeshToMesh(ntk::Mesh& mesh, pcl::PolygonMesh& polygon)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(polygon.cloud, cloud);
    pointCloudToMesh(mesh, cloud);
    mesh.faces.resize(polygon.polygons.size());
    foreach_idx(i, polygon.polygons)
    {
        const pcl::Vertices& vertices = polygon.polygons[i];
        ntk_assert(vertices.vertices.size() == 3, "Must be triangles!");
        ntk::Face& face = mesh.faces[i];
        face.indices[0] = vertices.vertices[0];
        face.indices[1] = vertices.vertices[1];
        face.indices[2] = vertices.vertices[2];
    }
}

void meshToPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,
                      const ntk::Mesh& mesh)
{
    cloud.points.resize(mesh.vertices.size());
    cloud.height = 1;
    cloud.width = cloud.points.size();
    foreach_idx(i, cloud.points)
    {
        cloud.points[i] = toPcl(mesh.vertices[i]);
    }
}

void meshToPointCloud(pcl::PointCloud<pcl::PointNormal>& cloud,
                      const ntk::Mesh& mesh)
{
    cloud.points.resize(mesh.vertices.size());
    cloud.height = 1;
    cloud.width = cloud.points.size();
    foreach_idx(i, cloud.points)
    {
        cloud.points[i] = toPcl(mesh.vertices[i], mesh.normals[i]);
    }
}

Eigen::Affine3f toPclCameraTransform(const Pose3D& pose)
{
    Eigen::Affine3f mat;
    cv::Mat1f T = pose.cvCameraTransform();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            mat(r,c) = T(r,c);
    return mat;
}

Eigen::Affine3f toPclInvCameraTransform(const Pose3D& pose)
{
    Eigen::Affine3f mat;
    cv::Mat1f T = pose.cvInvCameraTransform();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            mat(r,c) = T(r,c);
    return mat;
}


} // ntk
