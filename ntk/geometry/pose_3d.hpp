#ifndef NESTK_GEOMETRY_POSE_3D_HPP
# define NESTK_GEOMETRY_POSE_3D_HPP

# include <Eigen/Core>
# include <Eigen/Geometry>

namespace ntk
{

class EigenIsometry3dHolder
{
public:
    EigenIsometry3dHolder()
        : camera_transform (0)
        , inv_camera_transform (0)
        , projective_transform (0)
    {}

    Eigen::Isometry3d* camera_transform;
    Eigen::Isometry3d* inv_camera_transform;
    Eigen::Projective3d* projective_transform;
};

} // ntk

#endif // NESTK_GEOMETRY_POSE_3D_HPP
