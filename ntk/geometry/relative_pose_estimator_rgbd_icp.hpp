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


#ifndef NESTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_RGBD_ICP_HPP
#define NESTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_RGBD_ICP_HPP

#include "relative_pose_estimator_rgbd_icp.h"

#include <ntk/mesh/pcl_utils.h>

#include <ntk/geometry/transformation_estimation_rgbd.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#ifdef HAVE_PCL_GREATER_THAN_1_6_0
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#endif

#ifdef HAVE_PCL_GREATER_THAN_1_2_0
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#endif

namespace ntk
{

template <class PointT>
bool RelativePoseEstimatorRGBDICP<PointT> ::
computeRegistration(Pose3D& relative_pose,
                    PointCloudConstPtr source_cloud,
                    PointCloudConstPtr target_cloud,
                    PointCloudType& aligned_cloud)
{
#ifndef HAVE_PCL_GREATER_THAN_1_6_0
    ntk_dbg(0) << "Warning: cannot run RGBD-ICP because PCL version is < 1.7";
    return super::computeRegistration(relative_pose, source_cloud, target_cloud, aligned_cloud);
#else
    pcl::IterativeClosestPoint<PointT, PointT> reg;
    typedef pcl::registration::TransformationEstimationPointToPlaneLLS<PointT, PointT> PointToPlane;
    typedef TransformationEstimationRGBD<PointT, PointT> TransformRGBD;
    boost::shared_ptr<TransformRGBD> transform_rgbd (new TransformRGBD(&reg));
    if (target_points_3d && source_image_points)
        transform_rgbd->setColorFeatures(source_rgb_pose, *target_points_3d, *source_image_points);
    reg.setTransformationEstimation (transform_rgbd);

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

    reg.setMaximumIterations (m_max_iterations);
    reg.setTransformationEpsilon (1e-10);
    reg.setMaxCorrespondenceDistance (m_distance_threshold);
    reg.setRANSACOutlierRejectionThreshold(m_ransac_outlier_threshold);
    reg.setInputSource (source_cloud);
    reg.setInputTarget (target_cloud);
    reg.align (aligned_cloud);

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
#endif
}

} // ntk

#endif // NESTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_RGBD_ICP_HPP
