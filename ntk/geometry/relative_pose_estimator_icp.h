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
 */
class RelativePoseEstimatorICP : public RelativePoseEstimator
{
public:
    RelativePoseEstimatorICP() {}

public:
    void setReferenceImage(const RGBDImage& ref_image);

public:
    virtual bool estimateNewPose(const RGBDImage& image);
    virtual void reset() {}

protected:
    pcl::PointCloud<pcl::PointXYZ> m_ref_cloud;
};

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_ICP_H
