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
class RelativePoseEstimatorICP : public RelativePoseEstimator
{
public:
    RelativePoseEstimatorICP()
        : m_distance_threshold(0.05),
          m_voxel_leaf_size(0.005),
          m_max_iterations(20)
    {}

public:
    void setReferenceImage(const RGBDImage& ref_image);

    /*! Distance threshold to associate points. */
    void setDistanceThreshold(double th) { m_distance_threshold = th; }

    /*! Size of the leafs of the voxel grid. */
    void setVoxelSize(double s) { m_voxel_leaf_size = s; }

    /*! Set the maximal number of ICP iterations. */
    void setMaxIterations(int n) { m_max_iterations = n; }

public:
    virtual bool estimateNewPose(const RGBDImage& image);
    virtual void reset() {}

protected:
    pcl::PointCloud<pcl::PointXYZ> m_ref_cloud;
    double m_distance_threshold;
    double m_voxel_leaf_size;
    int m_max_iterations;
};

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_ICP_H
