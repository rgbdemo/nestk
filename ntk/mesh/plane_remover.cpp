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


#include "plane_remover.h"

void ntk::PlaneRemover::
removePlane(ntk::Mesh &mesh)
{
    Mesh new_mesh;
    foreach_idx(i, mesh.vertices)
    {
        cv::Point3f p = mesh.vertices[i];
        cv::Point3f normal = mesh.normals[i];

        if (m_plane.distanceToPlane(mesh.vertices[i]) > m_max_dist)
        {
            addVertex(mesh, new_mesh, i);
            continue;
        }

        if (acos(normal.dot(m_plane.normal())) > (m_max_normal_angle*M_PI/180.0))
        {
            addVertex(mesh, new_mesh, i);
            cv::Point3f plane_point = m_plane.intersectionWithLine(p, p - (cv::Point3f)m_plane.normal());
            new_mesh.vertices.push_back(plane_point);
            new_mesh.normals.push_back(m_plane.normal());
            new_mesh.colors.push_back(mesh.colors[i]);
            continue;
        }
    }
    mesh = new_mesh;
}

void ntk::PlaneRemover::
addVertex(const ntk::Mesh& source, ntk::Mesh &output, int index)
{
    output.vertices.push_back(source.vertices[index]);
    output.normals.push_back(source.normals[index]);
    output.colors.push_back(source.colors[index]);
}
