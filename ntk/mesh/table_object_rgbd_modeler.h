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
        InvalidVoxel = -1,
        UnknownVoxel = 0,
        BackgroundVoxel,
        MaybeBackgroundVoxel,
        ObjectVoxel,
        MaybeObjectVoxel,
    };

    /*! Store data related to current image processing. */
    struct CurrentImageData;

public:
    TableObjectRGBDModeler();
    virtual ~TableObjectRGBDModeler();

    /*! Create the voxel grid. */
    void initialize(const cv::Point3f& sizes, const cv::Point3f& offsets);

public:
    /*! Set whether depth filling should be used. */
    void setDepthFilling(bool useit) { m_depth_filling = useit; }

    /*! Set whether small structures should be filtered out. */
    void setRemoveSmallStructures(bool useit) { m_remove_small_structures = useit; }

    /*! Set the voxel grid resolution. */
    virtual void setResolution(float resolution) { m_resolution = resolution; }

    /*! Set the input table object detector and the cluster id to model. */
    void feedFromTableObjectDetector(const TableObjectDetector<pcl::PointXYZ>& detector, int cluster_id);

public:
    /*! Update the voxel grid using the given image. */
    virtual bool addNewView(const RGBDImage& image, Pose3D& depth_pose);

    /*! Compute a mesh out of the voxel grid. */
    virtual void computeMesh();

    /*! Only keep the outer surface of the computed mesh. */
    virtual void computeSurfaceMesh();

    /*! Compute final vertices colors by reprojecting the mesh on the color image. */
    virtual void computeAccurateVerticeColors();

    /*! Reset the voxel grid. */
    virtual void reset();

    /*! Returns the mesh volume in m^3. */
    float meshVolume() const;

private:
    /*! Compute 3D point coordinates from voxel grid coordinates. */
    cv::Point3f toRealWorld(const cv::Point3f& p) const;
    /*! Convert a 3D point to grid coordinates. */
    cv::Point3f toGridWorld(const cv::Point3f& p) const;

    /*! 3D morphological closing on the voxel grid. */
    void morphologicalClose();

    /*! 3D morphological opening on the voxel grid. */
    void morphologicalOpen();

    /*! Rank filter, elimite object voxels that do not have a least "rank" object neighbors. */
    void rankOpenFilter(int rank);

    /*! Build a new grid from given image and point cloud. */
    bool buildVoxelsFromNewView(CurrentImageData& d);

    /*! Initial fill of the voxel grid from new cluster. */
    void fillGridWithNewPoints(CurrentImageData& d);

    bool isOrMaybeIsObject(VoxelLabel l) { return l == ObjectVoxel || l == MaybeObjectVoxel; }

    void removeInconsistentObjectVoxels(CurrentImageData& d);

private:
    int m_cluster_id;
    cv::Mat_<uchar> m_voxels;
    cv::Mat_<cv::Vec3b> m_voxels_color;
    float m_resolution;
    float m_depth_margin;
    cv::Point3f m_offsets;
    cv::Point3f m_sizes;
    bool m_first_view;
    bool m_depth_filling;
    bool m_remove_small_structures;
    const std::vector<Point3f>* m_object_points;
    bool m_fed_from_table_detector;
};

} // ntk

#endif // NESTK_MESH_TABLE_OBJECT_RGBD_MODELER_H
