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


#ifndef RELATIVE_POSE_ESTIMATOR_ICP_HPP
#define RELATIVE_POSE_ESTIMATOR_ICP_HPP

#include "relative_pose_estimator_icp.h"

#include <ntk/mesh/pcl_utils.h>

#include <ntk/geometry/transformation_estimation_rgbd.h>

#include <pcl/registration/icp.h>
// #include <pcl/registration/gicp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#ifdef HAVE_PCL_GREATER_THAN_1_6_0
# include <pcl/registration/correspondence_rejection_surface_normal.h>
# include <pcl/registration/correspondence_rejection_var_trimmed.h>
#endif
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
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
        mesh1.saveToPlyFile("debug_mesh_source.ply");
        mesh2.saveToPlyFile("debug_mesh_target.ply");
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

    if (swap_source_and_target)
        relative_pose.invert();

    ntk_dbg_print(relative_pose.cvTranslation(), 1);

#if 0
    m_estimated_pose = m_target_pose;
    m_estimated_pose.applyTransformBefore(relative_pose);
    m_estimated_pose.applyTransformAfter(m_initial_pose);
#endif
    // Be careful. Poses are actually inverse camera transform in 3D space.
    // inverse(newpose) = inverse(relative_pose) * inverse(target_pose)
    m_estimated_pose = m_initial_pose;
    m_estimated_pose.applyTransformBefore(relative_pose.inverted());

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
    reg.setRANSACOutlierRejectionThreshold(m_ransac_outlier_threshold);
    reg.setMaxCorrespondenceDistance (m_distance_threshold);
    reg.setInputCloud (source_cloud);
    reg.setInputTarget (target_cloud);
    reg.align (aligned_cloud);

    if (0)
    {
        ntk::Mesh mesh1, mesh2;
        pointCloudToMesh(mesh1, aligned_cloud);
        pointCloudToMesh(mesh2, *target_cloud);
        mesh1.saveToPlyFile("debug_icp_1.ply");
        mesh2.saveToPlyFile("debug_icp_2.ply");
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
    pcl::IterativeClosestPoint<PointT, PointT> reg;
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT> PointToPlane;
    boost::shared_ptr<PointToPlane> point_to_plane (new PointToPlane);
    reg.setTransformationEstimation (point_to_plane);

    // typedef TransformationEstimationRGBD<PointT, PointT> TransformRGBD;
    // boost::shared_ptr<TransformRGBD> transform_rgbd (new TransformRGBD);
    // reg.setTransformationEstimation (transform_rgbd);

#ifdef HAVE_PCL_GREATER_THAN_1_6_0 // rejectors are not well supported before 1.7

    boost::shared_ptr<pcl::registration::CorrespondenceRejectorDistance> rejector_distance (new pcl::registration::CorrespondenceRejectorDistance);
    rejector_distance->setInputSource<PointT>(source_cloud);
    rejector_distance->setInputTarget<PointT>(target_cloud);
    rejector_distance->setMaximumDistance(m_distance_threshold);
    reg.addCorrespondenceRejector(rejector_distance);

    boost::shared_ptr<pcl::registration::CorrespondenceRejectorSurfaceNormal> rejector_normal (new pcl::registration::CorrespondenceRejectorSurfaceNormal);
    rejector_normal->setThreshold(cos(M_PI/4.f));
    rejector_normal->initializeDataContainer<PointT, PointT>();
    rejector_normal->setInputSource<PointT>(source_cloud);
    rejector_normal->setInputTarget<PointT>(target_cloud);
    rejector_normal->setInputNormals<PointT, PointT>(source_cloud);
    rejector_normal->setTargetNormals<PointT, PointT>(target_cloud);
    reg.addCorrespondenceRejector(rejector_normal);

    boost::shared_ptr<pcl::registration::CorrespondenceRejectorOneToOne> rejector_one_to_one (new pcl::registration::CorrespondenceRejectorOneToOne);
    reg.addCorrespondenceRejector(rejector_one_to_one);

    typedef pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> RejectorConsensusT;
    boost::shared_ptr<RejectorConsensusT> rejector_ransac (new RejectorConsensusT());
    rejector_ransac->setInputSource(source_cloud);
    rejector_ransac->setInputTarget(target_cloud);
    rejector_ransac->setInlierThreshold(m_ransac_outlier_threshold);
    rejector_ransac->setMaxIterations(100);
    reg.addCorrespondenceRejector(rejector_ransac);

    boost::shared_ptr<pcl::registration::CorrespondenceRejectorVarTrimmed> rejector_var_trimmed (new pcl::registration::CorrespondenceRejectorVarTrimmed());
    rejector_var_trimmed->setInputSource<PointT>(source_cloud);
    rejector_var_trimmed->setInputTarget<PointT>(target_cloud);
    rejector_var_trimmed->setMinRatio(0.1f);
    rejector_var_trimmed->setMaxRatio(0.75f);
    reg.addCorrespondenceRejector(rejector_var_trimmed);

    boost::shared_ptr<pcl::registration::CorrespondenceRejectorTrimmed> rejector_trimmed (new pcl::registration::CorrespondenceRejectorTrimmed());
    rejector_trimmed->setMinCorrespondences(static_cast<int>(0.1f * source_cloud->size()));
    rejector_trimmed->setOverlapRatio(0.5f);
    reg.addCorrespondenceRejector(rejector_trimmed);
#endif

#if 0
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
    reg.setRANSACOutlierRejectionThreshold(m_ransac_outlier_threshold);
#ifdef HAVE_PCL_GREATER_THAN_1_6_0
    reg.setInputSource (source_cloud);
#else
    reg.setInputCloud (source_cloud);
#endif
    reg.setInputTarget (target_cloud);
    reg.align (aligned_cloud);

    if (!reg.hasConverged())
    {
      ntk_dbg(1) << "ICP did not converge, ignoring.";
      return false;
    }

    if (0)
    {
        ntk::Mesh mesh1, mesh2;
        pointCloudToMesh(mesh1, aligned_cloud);
        pointCloudToMesh(mesh2, *target_cloud);
        mesh1.saveToPlyFile("debug_icp_1.ply");
        mesh2.saveToPlyFile("debug_icp_2.ply");
    }

#if 0
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

#if 0 // FIXME: broken in PCL 1.4
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
#endif

} // ntk

#endif // RELATIVE_POSE_ESTIMATOR_ICP_HPP
