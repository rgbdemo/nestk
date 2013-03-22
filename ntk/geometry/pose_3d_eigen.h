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
