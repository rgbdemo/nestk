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

#ifndef NTK_MESH_MESHGENERATOR_H
#define NTK_MESH_MESHGENERATOR_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/mesh/mesh.h>

namespace ntk
{

class Pose3D;

class MeshGenerator
{
public:
  enum MeshType { PointCloudMesh = 0, SurfelsMesh = 1, TriangleMesh = 2 };
public:
  MeshGenerator();

public:
  const ntk::Mesh& mesh() { return m_mesh; }

  void generateTriangleMesh(const RGBDImage& image, const Pose3D& depth_pose, const Pose3D& rgb_pose);
  void generatePointCloudMesh(const RGBDImage& image, const Pose3D& depth_pose, const Pose3D& rgb_pose);
  void generateSurfelsMesh(const RGBDImage& image, const Pose3D& depth_pose, const Pose3D& rgb_pose);
  void setUseColor(bool use_it) { m_use_color = use_it; }
  bool useColor() const { return m_use_color; }
  void setMeshType(MeshType type) { m_mesh_type = type; }
  void setResolutionFactor(double f);
  void setMaxNormalAngle(double angle_in_degrees) { m_max_normal_angle = angle_in_degrees; }
  void setMaxDeltaDepthBetweenEdges(float d) { m_max_delta_depth = d; }

public:
  void generate(const RGBDImage& image,
                const Pose3D& depth_pose = Pose3D(),
                const Pose3D& rgb_pose = Pose3D());

private:
  ntk::Mesh m_mesh;
  bool m_use_color;
  MeshType m_mesh_type;
  double m_resolution_factor;
  double m_max_normal_angle;
  float m_max_delta_depth;
};

float estimateErrorFromDepth(float depth, float max_depth_at_1m);

} // ntk

#endif // NTK_MESH_MESHGENERATOR_H
