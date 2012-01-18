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

#ifndef RELATIVE_POSE_ESTIMATOR_ICP_HPP
#define RELATIVE_POSE_ESTIMATOR_ICP_HPP

#include "relative_pose_estimator_icp.h"

#include <ntk/mesh/pcl_utils.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#ifdef HAVE_PCL_GREATER_THAN_1_2_0
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#endif

namespace ntk
{

template <class PointT>
bool RelativePoseEstimatorICP<PointT> :: preprocessClouds()
{
    if (m_target_cloud->points.size() < 1)
    {
        ntk_dbg(1) << "Target cloud was empty";
        return false;
    }

    PointCloudConstPtr target = m_target_cloud;
    PointCloudConstPtr source = m_source_cloud;

    ntk_dbg_print(target->points.size(), 1);
    ntk_dbg_print(source->points.size(), 1);

    m_filtered_source.reset(new pcl::PointCloud<PointT>());
    m_filtered_target.reset(new pcl::PointCloud<PointT>());

    PointCloudPtr temp_source (new pcl::PointCloud<PointT>());
    PointCloudPtr temp_target (new pcl::PointCloud<PointT>());

    pcl::PassThrough<PointT> filter;
    filter.setInputCloud (source);
    filter.filter (*temp_source);

    filter.setInputCloud (target);
    filter.filter (*temp_target);

    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize (m_voxel_leaf_size, m_voxel_leaf_size, m_voxel_leaf_size);

    grid.setInputCloud(temp_source);
    grid.filter(*m_filtered_source);

    grid.setInputCloud(temp_target);
    grid.filter(*m_filtered_target);

    ntk_dbg_print(temp_source->points.size(), 1);
    ntk_dbg_print(m_filtered_source->points.size(), 1);
    ntk_dbg_print(m_filtered_target->points.size(), 1);

    if (m_filtered_source->points.size() < 1 || m_filtered_target->points.size() < 1)
    {
        ntk_dbg(1) << "Warning: no remaining points after filtering.";
        return false;
    }

    if (m_initial_pose.isValid())
    {
        Eigen::Affine3f H = toPclInvCameraTransform(m_initial_pose);
        this->transformPointCloud(*m_filtered_source, *m_filtered_source, H);
    }

    if (m_target_pose.isValid())
    {
        Eigen::Affine3f H = toPclInvCameraTransform(m_target_pose);
        this->transformPointCloud(*m_filtered_target, *m_filtered_target, H);
    }

    return true;
}

template <class PointT>
bool RelativePoseEstimatorICP<PointT> :: estimateNewPose()
{
    bool ok = preprocessClouds();
    if (!ok)
        return false;

    if (0)
    {
        ntk::Mesh mesh1, mesh2;
        pointCloudToMesh(mesh1, *m_filtered_source);
        pointCloudToMesh(mesh2, *m_filtered_target);
        mesh1.saveToPlyFile("/tmp/debug_mesh_source.ply");
        mesh2.saveToPlyFile("/tmp/debug_mesh_target.ply");
    }

    // The source cloud should be smaller than the target one.
    bool swap_source_and_target = false;
    if (m_filtered_source->points.size() > 2.0 * m_filtered_target->points.size())
    {
        swap_source_and_target = true;
        std::swap(m_filtered_source, m_filtered_target);
    }

    Pose3D relative_pose;
    PointCloudType cloud_reg;
    ok = computeRegistration(relative_pose, m_filtered_source, m_filtered_target, cloud_reg);

    if (!ok)
        return false;

    if (!swap_source_and_target)
        relative_pose.invert();

    ntk_dbg_print(relative_pose.cvTranslation(), 1);

    m_estimated_pose = m_target_pose;
    m_estimated_pose.applyTransformBefore(relative_pose);
    m_estimated_pose.applyTransformAfter(m_initial_pose);

    return true;
}

template <class PointT>
void RelativePoseEstimatorICP<PointT> ::
transformPointCloud(PointCloudType& input,
                    PointCloudType& output,
                    Eigen::Affine3f& H)
{
    pcl::transformPointCloud(input, output, H);
}

template <class PointT>
bool RelativePoseEstimatorICP<PointT> ::
computeRegistration(Pose3D& relative_pose,
                    PointCloudConstPtr source_cloud,
                    PointCloudConstPtr target_cloud,
                    PointCloudType& aligned_cloud)
{
    pcl::IterativeClosestPoint<PointT, PointT> reg;
    reg.setMaximumIterations (m_max_iterations);
    reg.setTransformationEpsilon (1e-10);
    reg.setMaxCorrespondenceDistance (m_distance_threshold);
    reg.setInputCloud (m_filtered_source);
    reg.setInputTarget (m_filtered_target);
    reg.align (aligned_cloud);

    if (0)
    {
        ntk::Mesh mesh1, mesh2;
        pointCloudToMesh(mesh1, aligned_cloud);
        pointCloudToMesh(mesh2, *target_cloud);
        mesh1.saveToPlyFile("/tmp/debug_icp_1.ply");
        mesh2.saveToPlyFile("/tmp/debug_icp_2.ply");
    }

    if (!reg.hasConverged())
    {
      ntk_dbg(1) << "ICP did not converge, ignoring.";
      return false;
    }

    ntk_dbg_print(reg.getFitnessScore(), 1);

    Eigen::Matrix4f t = reg.getFinalTransformation ();
    cv::Mat1f T(4,4);
    //toOpencv(t,T);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r,c) = t(r,c);

    relative_pose.setCameraTransform(T);
    return true;
}

} // ntk

namespace ntk
{

template <class PointT>
bool RelativePoseEstimatorICPWithNormals<PointT> ::
computeRegistration(Pose3D& relative_pose,
                    PointCloudConstPtr source_cloud,
                    PointCloudConstPtr target_cloud,
                    PointCloudType& aligned_cloud)
{
#ifndef HAVE_PCL_GREATER_THAN_1_2_0
    return super::computeRegistration(relative_pose, source_cloud, target_cloud, aligned_cloud);
#else
    pcl::IterativeClosestPointNonLinear<PointT, PointT> reg;
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT> PointToPlane;
    boost::shared_ptr<PointToPlane> point_to_plane (new PointToPlane);
    reg.setTransformationEstimation (point_to_plane);

#if 1
    ntk::Mesh target_mesh;
    pointCloudToMesh(target_mesh, *target_cloud);
    target_mesh.saveToPlyFile("debug_target.ply");

    ntk::Mesh source_mesh;
    pointCloudToMesh(source_mesh, *source_cloud);
    source_mesh.saveToPlyFile("debug_source.ply");
#endif

    reg.setMaximumIterations (m_max_iterations);
    reg.setTransformationEpsilon (1e-10);
    reg.setMaxCorrespondenceDistance (m_distance_threshold);
    reg.setInputCloud (source_cloud);
    reg.setInputTarget (target_cloud);
    reg.align (aligned_cloud);

    if (!reg.hasConverged())
    {
      ntk_dbg(1) << "ICP did not converge, ignoring.";
      return false;
    }

#if 1
    ntk::Mesh debug_mesh;
    pointCloudToMesh(debug_mesh, aligned_cloud);
    debug_mesh.saveToPlyFile("debug_aligned.ply");
#endif

    ntk_dbg_print(reg.getFitnessScore(), 1);

    Eigen::Matrix4f t = reg.getFinalTransformation ();
    cv::Mat1f T(4,4);
    //toOpencv(t,T);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r,c) = t(r,c);

    relative_pose.setCameraTransform(T);
#endif
    return true;
}

template <class PointT>
void RelativePoseEstimatorICPWithNormals<PointT> ::
transformPointCloud(PointCloudType& input,
                    PointCloudType& output,
                    Eigen::Affine3f& H)
{
    pcl::transformPointCloudWithNormals(input, output, H);
}

} // ntk

namespace ntk
{

template <class PointT>
bool RelativePoseEstimatorGICP<PointT> ::
computeRegistration(Pose3D& relative_pose,
                    PointCloudConstPtr source_cloud,
                    PointCloudConstPtr target_cloud,
                    PointCloudType& aligned_cloud)
{
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> reg;
    reg.setMaximumIterations (m_max_iterations);
    reg.setTransformationEpsilon (1e-10);
    reg.setMaxCorrespondenceDistance (m_distance_threshold);
    reg.setInputCloud (source_cloud);
    reg.setInputTarget (target_cloud);
    reg.align (aligned_cloud);

    if (0)
    {
        ntk::Mesh mesh1, mesh2;
        pointCloudToMesh(mesh1, aligned_cloud);
        pointCloudToMesh(mesh2, *target_cloud);
        mesh1.saveToPlyFile("/tmp/debug_icp_1.ply");
        mesh2.saveToPlyFile("/tmp/debug_icp_2.ply");
    }

    if (!reg.hasConverged())
    {
      ntk_dbg(1) << "ICP did not converge, ignoring.";
      return false;
    }

    ntk_dbg_print(reg.getFitnessScore(), 1);

    Eigen::Matrix4f t = reg.getFinalTransformation ();
    cv::Mat1f T(4,4);
    //toOpencv(t,T);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r,c) = t(r,c);

    relative_pose.setCameraTransform(T);
    return true;
}

} // ntk

#endif // RELATIVE_POSE_ESTIMATOR_ICP_HPP
