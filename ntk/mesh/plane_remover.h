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
        : m_max_dist(0.01f),
          m_max_normal_angle(60)
    {}

public:
    void setInputPlane(const ntk::Plane& plane) { m_plane = plane; }
    void removePlane(ntk::Mesh& mesh);
    void setMaxDistance(float dist) { m_max_dist = dist; }

private:
    void addVertex(const Mesh& source, Mesh& output, int index);

private:
    Plane m_plane;
    float m_max_dist;
    float m_max_normal_angle;
};

} // ntk

#endif // NTK_MESH_PLANE_REMOVER_H
