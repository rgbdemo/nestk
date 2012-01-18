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

#ifndef NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_ICP_H
#define NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_ICP_H

#ifndef NESTK_USE_PCL
# define NESTK_USE_PCL 1 // for QTcreator.
#endif
#include "relative_pose_estimator.h"

namespace ntk
{

/*!
 * Estimate the relative 3D pose between a new image and the reference one.
 *
 * The return pose is such that the new image, if projected to 3D using
 * the estimated pose, will result into an aligned point cloud.
 *
 * This is based on the PCL library implementation of ICP.
 * The algorithm first downsample the image using a voxel grid filter.
 */
template <class PointT>
class RelativePoseEstimatorICP : public RelativePoseEstimatorFromPointClouds<PointT>
{
private:
    typedef RelativePoseEstimatorFromPointClouds<PointT> super;
    using super::m_target_cloud;
    using super::m_source_cloud;
    using super::m_initial_pose;
    using super::m_target_pose;
    using super::m_estimated_pose;

    typedef pcl::PointCloud<PointT> PointCloudType;
    typedef typename PointCloudType::ConstPtr PointCloudConstPtr;
    typedef typename PointCloudType::Ptr PointCloudPtr;

public:
    RelativePoseEstimatorICP()
        : m_distance_threshold(0.05),
          m_voxel_leaf_size(0.005),
          m_max_iterations(20)
    {}

public:
    /*! Distance threshold to associate points. */
    void setDistanceThreshold(double th) { m_distance_threshold = th; }

    /*! Size of the leafs of the voxel grid. */
    void setVoxelSize(double s) { m_voxel_leaf_size = s; }

    /*! Set the maximal number of ICP iterations. */
    void setMaxIterations(int n) { m_max_iterations = n; }

public:
    virtual bool estimateNewPose();
    virtual void transformPointCloud(PointCloudType& input,
                                     PointCloudType& output,
                                     Eigen::Affine3f& H);

protected:
    virtual bool preprocessClouds();

    virtual bool computeRegistration(Pose3D& relative_pose,
                                     PointCloudConstPtr source_cloud,
                                     PointCloudConstPtr target_cloud,
                                     PointCloudType& aligned_cloud);

protected:
    double m_distance_threshold;
    double m_voxel_leaf_size;
    int m_max_iterations;
    PointCloudPtr m_filtered_target;
    PointCloudPtr m_filtered_source;
};

template <class PointT>
class RelativePoseEstimatorICPWithNormals : public RelativePoseEstimatorICP<PointT>
{
    typedef RelativePoseEstimatorICP<PointT> super;
    typedef pcl::PointCloud<PointT> PointCloudType;
    typedef typename PointCloudType::ConstPtr PointCloudConstPtr;
    typedef typename PointCloudType::Ptr PointCloudPtr;

    using super::m_max_iterations;
    using super::m_distance_threshold;

protected:
    virtual bool computeRegistration(Pose3D& relative_pose,
                                     PointCloudConstPtr source_cloud,
                                     PointCloudConstPtr target_cloud,
                                     PointCloudType& aligned_cloud);

    virtual void transformPointCloud(PointCloudType& input,
                                     PointCloudType& output,
                                     Eigen::Affine3f& H);
};

template <class PointT>
class RelativePoseEstimatorGICP : public RelativePoseEstimatorICP<PointT>
{
    typedef RelativePoseEstimatorICP<PointT> super;
    typedef pcl::PointCloud<PointT> PointCloudType;
    typedef typename PointCloudType::ConstPtr PointCloudConstPtr;
    typedef typename PointCloudType::Ptr PointCloudPtr;

    using super::m_max_iterations;
    using super::m_distance_threshold;

protected:
    virtual bool computeRegistration(Pose3D& relative_pose,
                                     PointCloudConstPtr source_cloud,
                                     PointCloudConstPtr target_cloud,
                                     PointCloudType& aligned_cloud);
};

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_ICP_H
