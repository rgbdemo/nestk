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

#ifndef NTK_MESH_RGBD_MODELER_H
#define NTK_MESH_RGBD_MODELER_H

#include <ntk/core.h>
#include <ntk/camera/calibration.h>
#include <ntk/mesh/mesh.h>

namespace ntk
{

class Pose3D;

class RGBDModeler
{
public:
  RGBDModeler() : m_global_depth_offset(0) {}

public:
  virtual bool addNewView(const RGBDImage& image, Pose3D& relative_pose);
  virtual void computeMesh() {}
  virtual void computeSurfaceMesh() {}
  virtual void computeAccurateVerticeColors() {}
  const Mesh& currentMesh() const { return m_mesh; }
  const RGBDImage& lastImage() const { return m_last_image; }
  virtual void reset() { m_mesh.clear(); }
  void setGlobalDepthOffset(float offset) { m_global_depth_offset = offset; }
  virtual void setResolution(float resolution) {}
  virtual void setDepthMargin(float depth_margin) {}
  const ntk::Plane& supportPlane() const { return m_support_plane; }

protected:
  ntk::Mesh m_mesh;
  float m_global_depth_offset;
  ntk::Plane m_support_plane;
  ntk::RGBDImage m_last_image;
};

} // ntk

#endif // NTK_MESH_RGBD_MODELER_H
