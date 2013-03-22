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


#include "table_object_rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/detection/plane_estimator.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>
#include <ntk/image/color_model.h>

#include <ntk/mesh/mesh_renderer.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace pcl;
using cv::Point3f;
using cv::Vec3b;
using cv::Mat1f;
using cv::Vec3f;

namespace ntk
{

struct TableObjectRGBDModeler :: CurrentImageData
{
    // Current image being processed.
    const RGBDImage* image;

    // Corresponding PCL point cloud.
    PointCloud<PointXYZIndex> cloud;

    // Pose3D in depth camera frame.
    Pose3D depth_pose;

    // Pose3D in rgb camera frame.
    Pose3D rgb_pose;

    // Image after depth painting.
    RGBDImage painted_image;

    // Temporary images of the area to be painted.
    cv::Mat1b object_img;
    cv::Mat1f object_depth;
    cv::Mat3b object_color;

    // 2D region of interest around the object.
    cv::Rect roi;

    // 3D region of interest around the object.
    Rect3f roi_3d;
};

TableObjectRGBDModeler :: TableObjectRGBDModeler() : RGBDModeler(),
    m_cluster_id(0),
    m_resolution(0.003f),
    m_depth_margin(0.f),
    m_first_view(true),
    m_depth_filling(true),
    m_remove_small_structures(true),
    m_object_points(0),
    m_fed_from_table_detector(false)
{
    ntk_dbg_print(m_resolution, 1);
}

TableObjectRGBDModeler :: ~TableObjectRGBDModeler()
{
}

void TableObjectRGBDModeler :: feedFromTableObjectDetector(const TableObjectDetector<pcl::PointXYZ>& detector,
                                                           int cluster_id)
{
    m_support_plane = detector.plane();
    ntk_assert(cluster_id < detector.objectClusters().size(), "Invalid cluster.");
    m_object_points = &(detector.objectClusters()[cluster_id]);
    m_resolution = detector.voxelSize();
    m_fed_from_table_detector = true;
}

void TableObjectRGBDModeler :: initialize(const cv::Point3f& sizes,
                                          const cv::Point3f& offsets)
{
    ntk_dbg_print(m_resolution, 1);
    m_offsets = offsets;
    m_sizes = sizes;
    const int c_sizes[3] = {sizes.z/m_resolution, sizes.y/m_resolution, sizes.x/m_resolution};
    m_voxels.create(3, c_sizes);       
    for_all_drc(m_voxels) m_voxels(d,r,c) = UnknownVoxel;
    m_voxels_color.create(3, c_sizes);
    m_voxels_color = Vec3b(255,255,255);
}

bool TableObjectRGBDModeler :: addNewView(const RGBDImage& image, Pose3D& depth_pose)
{
    image.copyTo(m_last_image);
    TableObjectDetector<PointXYZ>* detector = 0;
    if (!m_fed_from_table_detector)
    {
        detector = new TableObjectDetector<PointXYZ>();
        detector->setObjectVoxelSize(m_resolution);
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        rgbdImageToPointCloud(*cloud, image, depth_pose);
        bool ok = detector->detect(cloud);
        if (!ok)
        {
            ntk_dbg(1) << "No cluster found.";
            return false;
        }
        int cluster_id = detector->getMostCentralCluster();
        feedFromTableObjectDetector(*detector, cluster_id);
    }

    ntk_assert(m_object_points, "You need to call setTableObjectDetector first.");

    CurrentImageData data;
    data.image = &image;
    data.depth_pose = depth_pose;
    data.rgb_pose = depth_pose;
    data.rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                                image.calibration()->R, image.calibration()->T);

    ntk::TimeCount tc("addNewView");

    PointCloud<PointXYZIndex>& cloud = data.cloud;
    rgbdImageToPointCloud(cloud, image, depth_pose);
    tc.elapsedMsecs("toPointCloud");

    bool ok = buildVoxelsFromNewView(data);
    if (!ok)
        return false;
    tc.elapsedMsecs(" -- first Build from Voxels");

    if (m_remove_small_structures)
    {
        morphologicalClose();
        rankOpenFilter(2);
        // openVolume();
    }

    tc.elapsedMsecs(" -- Remove small structures");

    computeMesh();
    computeSurfaceMesh();
    tc.stop("mesh computed");

    if (detector)
    {
        delete detector;
    }
    m_object_points = 0; // mask as not fed anymore.
    m_fed_from_table_detector = false;
    return true;
}

void TableObjectRGBDModeler :: reset()
{
    for_all_drc(m_voxels)
        m_voxels(d,r,c) = UnknownVoxel;
    RGBDModeler::reset();
    m_first_view = true;
}

void TableObjectRGBDModeler :: computeMesh()
{
    m_mesh.clear();

    // First pass fill up
    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != ObjectVoxel)
            continue;

        Vec3b color = m_voxels_color(d,r,c);

        Point3f p = toRealWorld(Point3f(c,r,d));
        float size_x = m_resolution;
#if 0
        for (++c; c < m_voxels.size[2] && m_voxels(d,r,c) == Object; ++c)
        {
            size_x += m_resolution;
            p.x += m_resolution/2.0;
        }
#endif
        m_mesh.addCube(p, Point3f(size_x,m_resolution,m_resolution), color);
    }
}

void TableObjectRGBDModeler :: computeSurfaceMesh()
{
    m_mesh.clear();
    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != ObjectVoxel)
            continue;

        bool is_edge = false;
        for (int dd=-1; !is_edge && dd<=1; ++dd)
            for (int dr=-1; !is_edge && dr<=1; ++dr)
                for (int dc=-1; !is_edge && dc<=1; ++dc)
                {
                    if (m_voxels(d+dd,r+dr,c+dc) != ObjectVoxel)
                        is_edge = true;
                }
        if (!is_edge)
            continue;

        Point3f p = toRealWorld(Point3f(c,r,d));
        float size_x = m_resolution;
        const Vec3b& color = m_voxels_color(d,r,c);
        m_mesh.addCube(p, Point3f(size_x,m_resolution,m_resolution), color);
    }    
}

void TableObjectRGBDModeler :: computeAccurateVerticeColors()
{
    std::fill(stl_bounds(m_mesh.colors), Vec3b(255,255,255));
    cv::Mat4b projected_image = toMat4b(m_last_image.rgb());
    MeshRenderer renderer(m_last_image.rgb().cols, m_last_image.rgb().rows);
    renderer.setMesh(m_mesh);
    renderer.setPose(*m_last_image.calibration()->rgb_pose);
    renderer.renderToImage(projected_image, MeshRenderer::NORMAL);
    foreach_idx(i, m_mesh.vertices)
    {
        Point3f p = m_last_image.calibration()->rgb_pose->projectToImage(m_mesh.vertices[i]);
        int r = ntk::math::rnd(p.y);
        int c = ntk::math::rnd(p.x);
        Vec3b color (255,255,255);
        if (is_yx_in_range(m_last_image.rgb(), r, c))
        {
            float d = renderer.depthBuffer()(r,c);
            if (flt_eq(p.z, d, 1e-2f))
            {
                color = bgr_to_rgb(m_last_image.rgb()(r,c));
            }
        }
        m_mesh.colors[i] = color;
    }
}

Point3f TableObjectRGBDModeler :: toGridWorld(const Point3f& p) const
{
    return Point3f((p.x - m_offsets.x - (m_resolution/2.0)) / m_resolution,
                   (p.y - m_offsets.y - (m_resolution/2.0)) / m_resolution,
                   (p.z - m_offsets.z - (m_resolution/2.0)) / m_resolution);
}

Point3f TableObjectRGBDModeler :: toRealWorld(const Point3f& p) const
{
    return Point3f(p.x*m_resolution + (m_resolution/2.0) + m_offsets.x,
                   p.y*m_resolution + (m_resolution/2.0) + m_offsets.y,
                   p.z*m_resolution + (m_resolution/2.0) + m_offsets.z);
}

void TableObjectRGBDModeler :: morphologicalClose()
{
    cv::Mat1b tmp; m_voxels.copyTo(tmp);
    for_all_drc(m_voxels)
    {
        if (m_voxels(d, r, c) != ObjectVoxel)
            continue;

        for (int dd = -1; dd <= 1; ++dd)
        for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc))
                continue;
            tmp(d+dd, r+dr, c+dc) = ObjectVoxel;
        }
    }

    for_all_drc(tmp)
    {
        if (tmp(d,r,c) != ObjectVoxel)
            continue;

        bool enabled = true;
        for (int dd = -1; enabled && dd <= 1; ++dd)
        for (int dr = -1; enabled && dr <= 1; ++dr)
        for (int dc = -1; enabled && dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(tmp, d+dd,r+dr,c+dc))
                continue;
            if (tmp(d+dd, r+dr, c+dc) != ObjectVoxel)
                enabled = false;
        }
        if (!enabled)
            m_voxels(d,r,c) = UnknownVoxel;
        else
            m_voxels(d,r,c) = ObjectVoxel;
    }
}

void TableObjectRGBDModeler :: morphologicalOpen()
{
    cv::Mat1b tmp; m_voxels.copyTo(tmp);

    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != ObjectVoxel)
            continue;

        bool enabled = true;
        for (int dd = -1; enabled && dd <= 1; ++dd)
        for (int dr = -1; enabled && dr <= 1; ++dr)
        for (int dc = -1; enabled && dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc)) continue;

            if (m_voxels(d+dd, r+dr, c+dc) != ObjectVoxel)
                enabled = false;
        }
        if (!enabled)
            tmp(d,r,c) = UnknownVoxel;
        else
            tmp(d,r,c) = ObjectVoxel;
    }

    m_voxels = UnknownVoxel;

    for_all_drc(tmp)
    {
        if (tmp(d, r, c) != ObjectVoxel)
            continue;

        for (int dd = -1; dd <= 1; ++dd)
        for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc))
                continue;
            m_voxels(d+dd, r+dr, c+dc) = ObjectVoxel;
        }
    }
}

void TableObjectRGBDModeler :: rankOpenFilter(int rank)
{
    cv::Mat1b tmp; m_voxels.copyTo(tmp);

    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != ObjectVoxel)
            continue;

        bool enabled = true;
        int nb_neighb = 0;
        for (int dd = -1; dd <= 1; ++dd)
        for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc)) continue;

            if (m_voxels(d+dd, r+dr, c+dc) == ObjectVoxel)
                ++nb_neighb;
        }
        if (nb_neighb < rank)
            tmp(d,r,c) = UnknownVoxel;
        else
            tmp(d,r,c) = ObjectVoxel;
    }

    m_voxels = tmp;
}


// Fill unknown voxels with given points, along with all voxels lying in the line between
// the observed voxel and the plane to fill the back face.
void TableObjectRGBDModeler :: fillGridWithNewPoints(CurrentImageData& d)
{
    const RGBDImage& image = *(d.image);
    const std::vector<Point3f>& object_points = *(m_object_points);
    const Pose3D& rgb_pose = d.rgb_pose;

    // Minimum for object points is 3mm from the plane.
    const float min_plane_dist = 0.003f;

    foreach_idx(i, object_points)
    {
        Point3f pobj = object_points[i];
        Point3f crd = toGridWorld(pobj);
        if (!is_zyx_in_range(m_voxels, crd.z, crd.y, crd.x))
            continue;

        if (m_voxels(crd.z, crd.y, crd.x) == ObjectVoxel)
            continue;

        // Mark the voxel.
        m_voxels(crd.z, crd.y, crd.x) = ObjectVoxel;

        // Fill voxels lying on the line towards the plane (extrusion).
        Point3f pplane = m_support_plane.intersectionWithLine(pobj, Vec3f(pobj)+m_support_plane.normal());
        pplane = pplane - Point3f(m_support_plane.normal()*min_plane_dist);
        Point3f v = pplane-pobj;
        float vnorm = norm(v);
        v *= 1.0f/vnorm;
        int istep = vnorm/m_resolution;
        for (int i = 0; i < istep; ++i)
        {
            Point3f p = pobj + v*i*m_resolution;
            Point3f rgb_p = rgb_pose.projectToImage(p);
            Vec3b color (255,255,255);
            if (is_yx_in_range(image.rgb(), rgb_p.y, rgb_p.x))
                color = image.rgb()(rgb_p.y, rgb_p.x);
            Point3f crd = toGridWorld(p);
            if (!is_zyx_in_range(m_voxels, crd.z, crd.y, crd.x))
                continue;
            if (m_voxels(crd.z, crd.y, crd.x) == UnknownVoxel)
            {
                m_voxels(crd.z, crd.y, crd.x) = ObjectVoxel;
                m_voxels_color(crd.z, crd.y, crd.x) = bgr_to_rgb(color);
            }
        }
    }
}

void TableObjectRGBDModeler :: removeInconsistentObjectVoxels(CurrentImageData& d)
{
    const RGBDImage& image = *(d.image);
    const Pose3D& depth_pose = d.depth_pose;

    const float depth_margin = 0.001f; // 1 mm
    const Mat1f& depth_im = image.depth();

    // Remove voxels object that project onto the background.
    for_all_drc(m_voxels)
    {
        if (!isOrMaybeIsObject((VoxelLabel)m_voxels(d, r, c)))
            continue;

        cv::Point3f p3d = toRealWorld(Point3f(c,r,d));
        Point3f p = depth_pose.projectToImage(p3d);

        if (!is_yx_in_range(depth_im, p.y, p.x))
            continue;

        // Depth value is further than voxel, eliminate voxel.
        if (depth_im(p.y, p.x) > (p.z + depth_margin))
        {
            m_voxels(d,r,c) = BackgroundVoxel;
        }
    }
}

bool TableObjectRGBDModeler :: buildVoxelsFromNewView(CurrentImageData& d)
{
    const RGBDImage& image = *(d.image);
    const Pose3D& depth_pose = d.depth_pose;
    const Pose3D& rgb_pose = d.rgb_pose;
    pcl::PointCloud<PointXYZIndex>& cloud = d.cloud;

    // FIXME: only one view supported right now.
    m_first_view = true;
    if (m_first_view)
    {
        // Initialize voxel gris for the working zone.
        ntk::Rect3f bbox = bounding_box(*m_object_points);
        Point3f working_zone = Point3f(0.5,0.5,0.5); // zone is 50x50x50cm centered of cluster center.
        initialize(working_zone, bbox.centroid()-0.5f*working_zone);
    }

    fillGridWithNewPoints(d);
    morphologicalClose();
    removeInconsistentObjectVoxels(d);
    return true;
}

float TableObjectRGBDModeler :: meshVolume() const
{
    int n_voxels = 0;
    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) == ObjectVoxel)
            n_voxels += 1;
    }
    ntk_dbg_print(n_voxels, 1);
    ntk_dbg_print(m_resolution, 1);
    ntk_dbg_print(n_voxels * (m_resolution)*(m_resolution)*(m_resolution), 1);
    return n_voxels * (m_resolution)*(m_resolution)*(m_resolution);
}

} // ntk
