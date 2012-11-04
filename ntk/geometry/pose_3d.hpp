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
        : camera_transform(0)
    {}

    Eigen::Isometry3d* camera_transform;
};

} // ntk

#endif // NESTK_GEOMETRY_POSE_3D_HPP
