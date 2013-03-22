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
