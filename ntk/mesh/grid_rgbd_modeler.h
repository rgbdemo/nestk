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

#ifndef NTK_MESH_GRID_RGBD_MODELER_H
# define NTK_MESH_GRID_RGBD_MODELER_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_calibration.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/rgbd_modeler.h>
#include <ntk/utils/opencv_utils.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ntk
{

class Pose3D;

class GridRGBDModeler : public RGBDModeler
{
public:
    GridRGBDModeler() : m_resolution(0.01), m_current_cloud(new pcl::PointCloud<pcl::PointXYZ>), m_colorize(false)
    {}

public:
    virtual float resolution() const { return m_resolution; }
    virtual void setResolution(float resolution) { m_resolution = resolution; reset(); }
    virtual int numPoints() const { return m_mesh.vertices.size(); }
    void setColorize(bool doit) { m_colorize = doit; }

public:
    virtual bool addNewView(const RGBDImage& image, Pose3D& depth_pose);
    virtual void reset();

protected:
    float m_resolution;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_current_cloud;
    bool m_colorize;
};
ntk_ptr_typedefs(GridRGBDModeler)

} // ntk

#endif // !NTK_MESH_GRID_RGBD_MODELER_H
