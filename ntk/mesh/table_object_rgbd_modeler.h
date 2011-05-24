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

#ifndef NESTK_MESH_TABLE_OBJECT_RGBD_MODELER_H
#define NESTK_MESH_TABLE_OBJECT_RGBD_MODELER_H

#ifndef NESTK_USE_PCL
# error "This file requires PCL support."
#endif

#include <ntk/core.h>
#include <ntk/camera/calibration.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/rgbd_modeler.h>
#include <ntk/geometry/plane.h>
#include <ntk/detection/table_object_detector.h>

namespace ntk
{

class TableObjectRGBDModeler : public RGBDModeler
{
private:
  enum VoxelLabel {
    Invalid = -1,
    Background = 0,
    Object = 1,
    Unknown = 2
  };

public:
  TableObjectRGBDModeler() : RGBDModeler(),
      m_first_view(true),
      m_depth_filling(true),
      m_remove_small_structures(true)
  {}

  void initialize(const cv::Point3f& sizes, const cv::Point3f& offsets,
                  float resolution, float depth_margin);

public:
  void setDepthFilling(bool useit) { m_depth_filling = useit; }
  void setRemoveSmallStructures(bool useit) { m_remove_small_structures = useit; }
  virtual void setResolution(float resolution) {} // { reset(); initialize(m_sizes, m_offsets, resolution, m_depth_margin); }
  virtual void setDepthMargin(float depth_margin) {} // { m_depth_margin = depth_margin; }

public:
  virtual bool addNewView(const RGBDImage& image, Pose3D& relative_pose);
  virtual void computeMesh();
  virtual void computeSurfaceMesh();
  virtual void computeAccurateVerticeColors();
  virtual void reset();

private:
  cv::Point3f toRealWorld(const cv::Point3f& p) const;
  cv::Point3f toGridWorld(const cv::Point3f& p) const;

  // 3D morphological on the voxel grid.
  void closeVolume();

  // 3D morphological opening on the voxel grid.
  void openVolume();

  // Rank filter: elimite object voxels that do not have a least k object neighbors.
  void rankOpenFilter(int rank);

  bool buildVoxelsFromNewView(const pcl::PointCloud<PointXYZIndex>& cloud, const RGBDImage& image, const Pose3D& depth_pose);
  void fillDepthImage(RGBDImage& painted_image, const RGBDImage& image, const Pose3D& depth_pose);

private:
  TableObjectDetector m_table_object_detector;
  cv::Mat_<uchar> m_voxels;
  cv::Mat_<cv::Vec3b> m_voxels_color;
  float m_resolution;
  float m_depth_margin;
  cv::Point3f m_offsets;
  cv::Point3f m_sizes;
  bool m_first_view;
  bool m_depth_filling;
  bool m_remove_small_structures;
};

} // ntk

#endif // NESTK_MESH_TABLE_OBJECT_RGBD_MODELER_H
