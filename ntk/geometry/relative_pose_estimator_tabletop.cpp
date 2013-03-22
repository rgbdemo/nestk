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


#include "relative_pose_estimator_tabletop.h"
#include "relative_pose_estimator_icp.h"

#include <ntk/utils/opencv_utils.h>

#include <pcl/common/centroid.h>
#include <pcl/registration/transforms.h>

#include <Eigen/Geometry>

bool ntk::RelativePoseEstimatorTableTop::
estimateNewPose()
{
    Eigen::Vector4f source_centroid;
    pcl::compute3DCentroid (*m_source_cloud, source_centroid);

    Eigen::Vector4f target_centroid;
    pcl::compute3DCentroid (*m_target_cloud, target_centroid);

    cv::Vec3f t (target_centroid[0]-source_centroid[0],
                 target_centroid[1]-source_centroid[1],
                 target_centroid[2]-source_centroid[2]);
    // FIXME: add a small perturbation to avoid local minimas.
    t[0] += 0.01f;
    t[1] += 0.01f;

    m_estimated_pose.applyTransformAfter(t, cv::Vec3f(0,0,0));

    // FIXME: use apply transform.
    pcl::PointCloud<pcl::PointXYZ>::Ptr centered_source(new pcl::PointCloud<pcl::PointXYZ>(*m_source_cloud));
    foreach_idx(i, *centered_source)
    {
        centered_source->points[i].x += t[0];
        centered_source->points[i].y += t[1];
        centered_source->points[i].z += t[2];
    }

    ntk_dbg_print(m_estimated_pose.cvEulerRotation(), 1);

    RelativePoseEstimatorICP<pcl::PointXYZ> icp_estimator;
    icp_estimator.setMaxIterations(100);
    icp_estimator.setDistanceThreshold(0.05);
    icp_estimator.setTargetCloud(m_target_cloud);
    icp_estimator.setSourceCloud(centered_source); // FIXME: Find out whether the removal of the (deep-copying) centered_source.makeShared() call sped things up.
#if 0
    pcl::transformPointCloud (centered_source,
                              centered_source,
                              (Eigen::Affine3f)Eigen::AngleAxisf(10, Eigen::Vector3f::UnitY()));
#endif
    icp_estimator.estimateNewPose();

    Pose3D icp_image_pose = icp_estimator.estimatedSourcePose();
    icp_image_pose.invert();
    m_estimated_pose.applyTransformAfter(icp_image_pose);
    ntk_dbg_print(m_estimated_pose.cvEulerRotation(), 1);
    return true;
}
