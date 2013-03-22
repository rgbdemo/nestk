
#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <ntk/mesh/mesh.h>
#include <ntk/mesh/pcl_utils.h>

#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>

using namespace ntk;

namespace pcl {

class EarClipping2
{
public:
    void setInputPolygonMesh(pcl::PolygonMeshConstPtr polygon)
    {
        polygon_ = polygon;
        fromROSMsg(polygon->cloud, points_);
    }

    void triangulate(PolygonMesh& output)
    {
        output.polygons.clear();
        output.cloud = polygon_->cloud;
        for (int i = 0; i < polygon_->polygons.size(); ++i)
        {
            triangulate(polygon_->polygons[i], output);
        }
    }

protected:
    void triangulate(const Vertices& vertices, PolygonMesh& output)
    {
        const int n_vertices = vertices.vertices.size();

        if (n_vertices <= 3)
        {
            output.polygons.push_back(vertices);
            return;
        }

        std::vector<uint32_t> remaining_vertices(n_vertices);
        if (area(vertices.vertices) > 0) // clockwise?
        {
            remaining_vertices = vertices.vertices;
        }
        else
        {
            for (int v = 0; v < n_vertices; v++)
                remaining_vertices[v] = vertices.vertices[n_vertices - 1 - v];
        }

        // Avoid closed loops.
        if (remaining_vertices.front() == remaining_vertices.back())
            remaining_vertices.erase(remaining_vertices.end()-1);

        // null_iterations avoids infinite loops if the polygon is not simple.
        for (int u = remaining_vertices.size() - 1, null_iterations = 0;
             remaining_vertices.size() > 2 && null_iterations < remaining_vertices.size()*2;
             ++null_iterations, u=(u+1)%remaining_vertices.size())
        {
            int v = (u + 1) % remaining_vertices.size();
            int w = (u + 2) % remaining_vertices.size();

            if (isEar(u, v, w, remaining_vertices))
            {
                Vertices triangle;
                triangle.vertices.resize(3);
                triangle.vertices[0] = remaining_vertices[u];
                triangle.vertices[1] = remaining_vertices[v];
                triangle.vertices[2] = remaining_vertices[w];
                output.polygons.push_back(triangle);
                remaining_vertices.erase(remaining_vertices.begin()+v);
                null_iterations = 0;
            }
        }
        for (int i = 0; i < remaining_vertices.size(); ++i)
            ntk_dbg_print(remaining_vertices[i], 1);
    }

    float area(const std::vector<uint32_t>& vertices)
    {
        int n = vertices.size();
        float area = 0.0f;
        for (int prev = n - 1, cur = 0; cur < n; prev = cur++)
        {
            PointXY prev_p = toPointXY(points_.points[vertices[prev]]);
            PointXY cur_p = toPointXY(points_.points[vertices[cur]]);
            area += crossProduct(prev_p, cur_p);
        }
        return area * 0.5f;
    }

    bool isEar(int u, int v, int w, const std::vector<uint32_t>& vertices)
    {
        PointXY p_u = toPointXY(points_.points[vertices[u]]);
        PointXY p_v = toPointXY(points_.points[vertices[v]]);
        PointXY p_w = toPointXY(points_.points[vertices[w]]);

        // Avoid flat triangles.
        // FIXME: what happens if all the triangles are flat in the X-Y axis?
        const float eps = 1e-15;
        PointXY p_uv = difference(p_v, p_u);
        PointXY p_uw = difference(p_w, p_u);
        if (crossProduct(p_uv, p_uw) < eps)
        {
            ntk_dbg(1) << cv::format("FLAT: (%d, %d, %d)", u, v, w);
            return false;
        }

        // Check if any other vertex is inside the triangle.
        for (int k = 0; k < vertices.size(); k++)
        {
            if ((k == u) || (k == v) || (k == w))
                continue;
            PointXY p = toPointXY(points_.points[vertices[k]]);
            if (isInsideTriangle(p_u, p_v, p_w, p))
                return false;
        }
        return true;
    }

    bool isInsideTriangle(const PointXY& u, const PointXY& v, const PointXY& w,
                          const PointXY& p)
    {
        PointXY vw = difference(w, v);
        PointXY wu = difference(u, w);
        PointXY uv = difference(v, u);
        PointXY up = difference(p, u);
        PointXY vp = difference(p, v);
        PointXY wp = difference(p, w);

        // Check first side.
        if (crossProduct(vw, vp) < 0)
            return false;

        // Check second side.
        if (crossProduct(uv, up) < 0)
            return false;

        // Check third side.
        if (crossProduct(wu, wp) < 0)
            return false;

        return true;
    }

    PointXY toPointXY(const PointXYZ& p) const
    { PointXY r; r.x = p.x; r.y = p.y; return r; }

    float crossProduct(const PointXY& p1, const PointXY& p2) const
    { return (p1.x*p2.y) - (p1.y*p2.x); }

    PointXY difference(const PointXY& p1, const PointXY& p2) const
    { PointXY r; r.x = p1.x - p2.x; r.y = p1.y - p2.y; return r; }

private:
    PolygonMeshConstPtr polygon_;
    PointCloud<PointXYZ> points_;
};

} // pcl

int main (int argc, char** argv)
{
    ntk::Mesh mesh;
    mesh.loadFromPlyFile("plane.ply");

    pcl::Vertices face;
    foreach_idx(i, mesh.vertices)
            face.vertices.push_back(i);
    face.vertices.push_back(0);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = mesh.vertices.size();
    cloud.height = 1;
    foreach_idx(i, mesh.vertices)
            cloud.points.push_back(toPcl(mesh.vertices[i]));

    pcl::PolygonMeshPtr polygon (new pcl::PolygonMesh());
    pcl::toROSMsg(cloud, polygon->cloud);
    polygon->polygons.push_back(face);
    pcl::EarClipping2 clipper;
    clipper.setInputPolygonMesh(polygon);
    pcl::PolygonMesh triangles;
    clipper.triangulate(triangles);

    polygonMeshToMesh(mesh, triangles);
    mesh.saveToPlyFile("plane_output.ply");
}
