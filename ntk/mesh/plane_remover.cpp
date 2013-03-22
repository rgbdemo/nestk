
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
