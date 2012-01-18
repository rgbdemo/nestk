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

#ifndef NTK_MESH_PLANE_REMOVER_H
#define NTK_MESH_PLANE_REMOVER_H

#include <ntk/geometry/plane.h>
#include <ntk/mesh/mesh.h>

namespace ntk
{

class PlaneRemover
{
public:
    PlaneRemover()
        : m_max_dist(0.01),
          m_max_normal_angle(60)
    {}

public:
    void setInputPlane(const ntk::Plane& plane) { m_plane = plane; }
    void removePlane(ntk::Mesh& mesh);

private:
    void addVertex(const Mesh& source, Mesh& output, int index);

private:
    Plane m_plane;
    float m_max_dist;
    float m_max_normal_angle;
};

} // ntk

#endif // NTK_MESH_PLANE_REMOVER_H
