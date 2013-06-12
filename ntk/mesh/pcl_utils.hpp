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


#ifndef NTK_MESH_PCL_UTILS_HPP
#define NTK_MESH_PCL_UTILS_HPP

#include <ntk/camera/rgbd_image.h>

#include "pcl_utils.h"

#include <pcl/filters/passthrough.h>

namespace ntk
{

template <class PointT>
void vectorToPointCloud(pcl::PointCloud<PointT>& cloud,
                        const std::vector<cv::Point3f>& points,
                        const std::vector<int>& indices)
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
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud, const RGBDImage& image, bool keep_dense)
{
    if (!image.calibration())
        ntk_throw_exception("No calibration data in image.");

    rgbdImageToPointCloud(cloud, image, *image.calibration()->depth_pose, 1.0, keep_dense);
}

template <class PointT>
void rgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud,
                           const RGBDImage& image,
                           const Pose3D& pose,
                           int subsampling_factor,
                           bool keep_dense)
{
    PointT nan_point;
    nan_point.getVector4fMap().setConstant (std::numeric_limits<float>::quiet_NaN());

    if (keep_dense)
    {
        cloud.width = image.depth().cols / subsampling_factor;
        cloud.height = image.depth().rows / subsampling_factor;
        cloud.points.resize(cloud.width*cloud.height, nan_point);
        cloud.is_dense = true;
    }
    else
    {
        cloud.clear();
    }

    for (int r = 0, cloud_r = 0; r < image.depth().rows; r += subsampling_factor, ++cloud_r)
    for (int c = 0, cloud_c = 0; c < image.depth().cols; c += subsampling_factor, ++cloud_c)
    {
        float d = image.depth()(r,c);
        bool mask_ok = !image.depthMask().data || image.depthMask()(r,c);
        if (d < 1e-5 || !mask_ok)
            continue;
        cv::Point3f p = pose.unprojectFromImage(cv::Point2f(c,r),d);
        PointT pcl_p;
        pcl_p.x = p.x;
        pcl_p.y = p.y;
        pcl_p.z = p.z;
        if (keep_dense)
            cloud.points[cloud_r*cloud.width+cloud_c] = pcl_p;
        else
            cloud.push_back(pcl_p);
        }   
}

template <>
void rgbdImageToPointCloud(pcl::PointCloud<pcl::PointNormal>& cloud,
                           const RGBDImage& image,
                           const Pose3D& pose,
                           int subsampling_factor,
                           bool keep_dense)
{
    typedef pcl::PointNormal PointT;

    PointT nan_point;
    nan_point.getVector4fMap().setConstant (std::numeric_limits<float>::quiet_NaN());

    if (keep_dense)
    {
        cloud.width = image.depth().cols / subsampling_factor;
        cloud.height = image.depth().rows / subsampling_factor;
        cloud.points.resize(cloud.width*cloud.height, nan_point);
        cloud.is_dense = true;
    }
    else
    {
        cloud.clear();
    }

    Pose3D rotation_pose;
    rotation_pose.applyTransformAfter(cv::Vec3f(0,0,0), pose.cvEulerRotation());

    for (int r = 0, cloud_r = 0; r < image.depth().rows; r += subsampling_factor, ++cloud_r)
    for (int c = 0, cloud_c = 0; c < image.depth().cols; c += subsampling_factor, ++cloud_c)
    {
        float d = image.depth()(r,c);

        bool mask_ok = !image.depthMask().data || image.depthMask()(r,c);
        if (d < 1e-5 || !mask_ok)
            continue;

        if (!image.isValidNormal(r,c))
            continue;

        cv::Point3f p = pose.unprojectFromImage(cv::Point2f(c,r), d);
        cv::Vec3f normal = image.normal()(r,c);
        cv::Vec3f n = rotation_pose.invCameraTransform(normal);
        PointT pcl_p;
        pcl_p.x = p.x;
        pcl_p.y = p.y;
        pcl_p.z = p.z;
        pcl_p.normal_x = n[0];
        pcl_p.normal_y = n[1];
        pcl_p.normal_z = n[2];

        if (keep_dense)
            cloud.points[cloud_r*cloud.width+cloud_c] = pcl_p;
        else
            cloud.push_back(pcl_p);
        }
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

template <>
void pointCloudToMesh(ntk::Mesh& mesh,
                      const pcl::PointCloud<pcl::PointNormal>& cloud)
{
    mesh.clear();
    mesh.vertices.resize(cloud.size());
    mesh.normals.resize(cloud.size());
    foreach_idx(i, cloud.points)
    {
        mesh.vertices[i] = cv::Point3f(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        mesh.normals[i] = cv::Point3f(cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z);
    }
}

template <class PointT>
void sampledRgbdImageToPointCloud(pcl::PointCloud<PointT>& cloud,
                                  const RGBDImage& image,
                                  const Pose3D& pose,
                                  int n_samples)
{
    cloud.width = n_samples;
    cloud.height = 1;
    cloud.points.resize(n_samples);
    cv::RNG rng;

    int i = 0;
    while (i < n_samples)
    {
        int r = rng(image.depth().rows);
        int c = rng(image.depth().cols);
        if (!image.depthMask()(r,c) || image.depth()(r,c) < 1e-5)
            continue;
        cv::Point3f p = pose.unprojectFromImage(cv::Point2f(c,r), image.depth()(r,c));
        cloud.points[i].x = p.x;
        cloud.points[i].y = p.y;
        cloud.points[i].z = p.z;
        i += 1;
    }
}

template <>
void sampledRgbdImageToPointCloud(pcl::PointCloud<pcl::PointNormal>& cloud,
                                  const RGBDImage& image,
                                  const Pose3D& pose,
                                  int n_samples)
{
    cloud.width = n_samples;
    cloud.height = 1;
    cloud.points.resize(n_samples);
    cv::RNG rng;

    Pose3D rotation_pose;
    rotation_pose.applyTransformAfter(cv::Vec3f(0,0,0), pose.cvEulerRotation());

    int i = 0;
    while (i < n_samples)
    {
        int r = rng(image.depth().rows);
        int c = rng(image.depth().cols);
        if (!image.depthMask()(r,c) || image.depth()(r,c) < 1e-5)
            continue;

        if (!image.isValidNormal(r,c))
            continue;

        cv::Point3f p = pose.unprojectFromImage(cv::Point2f(c,r), image.depth()(r,c));
        cloud.points[i].x = p.x;
        cloud.points[i].y = p.y;
        cloud.points[i].z = p.z;

        cv::Vec3f normal = image.normal()(r,c);
        cv::Vec3f n = rotation_pose.invCameraTransform(normal);
        cloud.points[i].normal_x = n[0];
        cloud.points[i].normal_y = n[1];
        cloud.points[i].normal_z = n[2];

        i += 1;
    }
}

template <class PointT>
void removeNan(pcl::PointCloud<PointT>& clean_cloud, typename pcl::PointCloud<PointT>::ConstPtr source_cloud)
{
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(source_cloud);
    pass.filter(clean_cloud);
}

} // ntk

#endif // NTK_MESH_PCL_UTILS_HPP
