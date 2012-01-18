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

#ifndef NTK_MESH_SURFELS_RGBD_MODELER_H
#define NTK_MESH_SURFELS_RGBD_MODELER_H

#include <ntk/core.h>
#include <ntk/camera/calibration.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/rgbd_modeler.h>

namespace ntk
{

class SurfelsRGBDModeler : public RGBDModeler
{
public:
    SurfelsRGBDModeler() : m_min_views(2), m_resolution(0.005), m_update_max_normal_angle(60)
    {}

public:
    void setMinViewsPerSurfel(int n) { m_min_views = n; }
    void setResolution(float r);
    virtual float resolution() const { return m_resolution; }
    virtual int numPoints() const { return m_surfels.size(); }

public:
    virtual bool addNewView(const RGBDImage& image, Pose3D& depth_pose);
    virtual void computeMesh();

    virtual void reset() { RGBDModeler::reset(); m_surfels.clear(); }    

protected:
    struct Cell
    {
        unsigned x, y, z;

        Cell(unsigned x, unsigned y, unsigned z) : x(x), y(y), z(z) {}
        Cell() : x(0), y(0), z(0) {}

        bool operator==(const Cell& rhs) const { return !(*this < rhs) && !(rhs < *this); }
        bool operator!=(const Cell& rhs) const { return !(*this == rhs); }

        bool operator<(const Cell& rhs) const
        {
            if (x < rhs.x) return true;
            if (x > rhs.x) return false;
            if (y < rhs.y) return true;
            if (y > rhs.y) return false;
            if (z < rhs.z) return true;
            if (z > rhs.z) return false;
            return false;
        }
    };

    typedef std::multimap<Cell, Surfel> SurfelMap;

    Cell worldToCell(const cv::Point3f& p)
    { return Cell(p.x / m_resolution, p.y / m_resolution, p.z / m_resolution); }

    cv::Point3f cellToWorld(const Cell& c)
    { return cv::Point3f(c.x*m_resolution, c.y*m_resolution, c.z*m_resolution); }

    /*! Return the compatibility threshold in function of the depth value. */
    float getCompatibilityDistance(float depth) const;

protected:
    bool mergeToLeftSurfel(Surfel& dest, const Surfel& src);
    float computeSurfelRadius(float depth, float camera_z, double mean_focal);
    bool normalsAreCompatible(const Surfel& lhs, const Surfel& rhs);

protected:
    SurfelMap m_surfels;
    int m_min_views;
    float m_resolution;
    float m_update_max_normal_angle;

    friend class LocalPlaneSegmentor;
};
ntk_ptr_typedefs(SurfelsRGBDModeler)

class ICPSurfelsRGBDModeler : public SurfelsRGBDModeler
{
public:
    ICPSurfelsRGBDModeler() : SurfelsRGBDModeler()
    {}

public:
    void setMinViewsPerSurfel(int n) { m_min_views = n; }

public:
    virtual bool addNewView(const RGBDImage& image, Pose3D& relative_pose);

protected:
    Pose3D fixRelativePose(const RGBDImage& image, const Pose3D& relative_pose);

private:
    ntk::Mesh m_point_cloud;
};

} // ntk

#endif // NTK_MESH_SURFELS_RGBD_MODELER_H
