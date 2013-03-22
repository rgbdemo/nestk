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

#ifndef NESTK_GEOMETRY_POSE_3D_EIGEN_H
#define NESTK_GEOMETRY_POSE_3D_EIGEN_H

#include <ntk/geometry/pose_3d.h>
#include <ntk/geometry/pose_3d.hpp>

namespace ntk
{

class EigenPose3DAccessor
{
public:
    EigenPose3DAccessor(const Pose3D& pose)
        : pose(pose)
    {}

    /*! Returns the camera transform as an Eigen double 4x4 matrix. */
    const Eigen::Isometry3d& eigenCameraTransform() const
    {
        EigenIsometry3dHolder holder;
        pose.getEigenCameraTransform(&holder);
        return *holder.camera_transform;
    }

    /*! Returns the inverse camera transform as an Eigen double 4x4 matrix. */
    const Eigen::Isometry3d& eigenInvCameraTransform() const
    {
        EigenIsometry3dHolder holder;
        pose.getEigenCameraTransform(&holder);
        return *holder.inv_camera_transform;
    }

    /*! Returns the camera transform as an Eigen double 4x4 matrix. */
    const Eigen::Projective3d& eigenProjectiveTransform() const
    {
        EigenIsometry3dHolder holder;
        pose.getEigenCameraTransform(&holder);
        return *holder.projective_transform;
    }

    const Pose3D& pose;
};

}

#endif // NESTK_GEOMETRY_POSE_3D_EIGEN_H
