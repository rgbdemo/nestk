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

#include "table_object_rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/detection/plane_estimator.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>
#include <ntk/image/color_model.h>

#include <ntk/mesh/mesh_renderer.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace pcl;

namespace ntk
{

void TableObjectRGBDModeler :: initialize(const cv::Point3f& sizes,
                                          const cv::Point3f& offsets,
                                          float resolution,
                                          float depth_margin)
{
    m_resolution = resolution;
    m_depth_margin = depth_margin;
    m_offsets = offsets;
    m_sizes = sizes;
    const int c_sizes[3] = {sizes.z/m_resolution, sizes.y/m_resolution, sizes.x/m_resolution};
    m_voxels.create(3, c_sizes);       
    for_all_drc(m_voxels) m_voxels(d,r,c) = Unknown;
    m_voxels_color.create(3, c_sizes);
    m_voxels_color = Vec3b(255,255,255);
}

void TableObjectRGBDModeler :: computeMesh()
{
    m_mesh.clear();

    // First pass fill up
    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != Object)
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
        if (m_voxels(d,r,c) != Object)
            continue;

        bool is_edge = false;
        for (int dd=-1; !is_edge && dd<=1; ++dd)
            for (int dr=-1; !is_edge && dr<=1; ++dr)
                for (int dc=-1; !is_edge && dc<=1; ++dc)
                {
                    if (m_voxels(d+dd,r+dr,c+dc) != Object)
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
            if (flt_eq(p.z, d, 1e-2))
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

void TableObjectRGBDModeler :: closeVolume()
{
    cv::Mat1b tmp; m_voxels.copyTo(tmp);
    for_all_drc(m_voxels)
    {
        if (m_voxels(d, r, c) != Object)
            continue;

        for (int dd = -1; dd <= 1; ++dd)
        for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc))
                continue;
            tmp(d+dd, r+dr, c+dc) = Object;
        }
    }

    for_all_drc(tmp)
    {
        if (tmp(d,r,c) != Object)
            continue;

        bool enabled = true;
        for (int dd = -1; enabled && dd <= 1; ++dd)
        for (int dr = -1; enabled && dr <= 1; ++dr)
        for (int dc = -1; enabled && dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(tmp, d+dd,r+dr,c+dc))
                continue;
            if (tmp(d+dd, r+dr, c+dc) != Object)
                enabled = false;
        }
        if (!enabled)
            m_voxels(d,r,c) = Unknown;
        else
            m_voxels(d,r,c) = Object;
    }
}

void TableObjectRGBDModeler :: openVolume()
{
    cv::Mat1b tmp; m_voxels.copyTo(tmp);

    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != Object)
            continue;

        bool enabled = true;
        for (int dd = -1; enabled && dd <= 1; ++dd)
        for (int dr = -1; enabled && dr <= 1; ++dr)
        for (int dc = -1; enabled && dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc)) continue;

            if (m_voxels(d+dd, r+dr, c+dc) != Object)
                enabled = false;
        }
        if (!enabled)
            tmp(d,r,c) = Unknown;
        else
            tmp(d,r,c) = Object;
    }

    m_voxels = Unknown;

    for_all_drc(tmp)
    {
        if (tmp(d, r, c) != Object)
            continue;

        for (int dd = -1; dd <= 1; ++dd)
        for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc))
                continue;
            m_voxels(d+dd, r+dr, c+dc) = Object;
        }
    }
}

void TableObjectRGBDModeler :: rankOpenFilter(int rank)
{
    cv::Mat1b tmp; m_voxels.copyTo(tmp);

    for_all_drc(m_voxels)
    {
        if (m_voxels(d,r,c) != Object)
            continue;

        bool enabled = true;
        int nb_neighb = 0;
        for (int dd = -1; dd <= 1; ++dd)
        for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc)
        {
            if (!is_zyx_in_range(m_voxels, d+dd,r+dr,c+dc)) continue;

            if (m_voxels(d+dd, r+dr, c+dc) == Object)
                ++nb_neighb;
        }
        if (nb_neighb < rank)
            tmp(d,r,c) = Unknown;
        else
            tmp(d,r,c) = Object;
    }

    m_voxels = tmp;
}

bool TableObjectRGBDModeler :: buildVoxelsFromNewView(const pcl::PointCloud<PointXYZIndex>& cloud,
                                                      const RGBDImage& image,
                                                      const Pose3D& depth_pose)
{
    // object must be closer than 2 cm from the plane.
    const float max_dist_to_plane_threshold = 0.02;

    bool ok = m_table_object_detector.detect(cloud);
    if (!ok)
        return false;

    if (m_table_object_detector.objectClusters().size() < 1)
        return false;

    m_support_plane = m_table_object_detector.plane();

    int selected_object = -1;
    float min_x = FLT_MAX;
    for (int i = 0; i < m_table_object_detector.objectClusters().size(); ++i)
    {
        const std::vector<Point3f>& object_points = m_table_object_detector.objectClusters()[i];
        float min_dist_to_plane = FLT_MAX;
        for (int j = 0; j < object_points.size(); ++j)
        {
            Point3f pobj = object_points[j];
            min_dist_to_plane = std::min(m_support_plane.distanceToPlane(pobj), min_dist_to_plane);
        }

        if (min_dist_to_plane > max_dist_to_plane_threshold)
            continue;

        ntk::Rect3f bbox = bounding_box(object_points);        
        if (std::abs(bbox.centroid().x) < min_x)
        {
            min_x = std::abs(bbox.centroid().x);
            selected_object = i;
        }
    }

    if (selected_object < 0)
        return false;

    const std::vector<Point3f>& object_points = m_table_object_detector.objectClusters()[selected_object];
    // FIXME: only one view supported right now.
    m_first_view = true;
    if (m_first_view)
    {
        ntk::Rect3f bbox = bounding_box(object_points);
        m_mesh.addCube(bbox.centroid(), Point3f(bbox.width, bbox.height, bbox.depth));

        ntk_dbg_print(bbox.centroid(), 1);

        Point3f working_zone = Point3f(0.5,0.5,0.5);
        initialize(working_zone, bbox.centroid()-0.5f*working_zone, 0.003 /*res*/, 0.000);
    }

    Pose3D rgb_pose = depth_pose;
    rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                           image.calibration()->R, image.calibration()->T);
    const Mat1f& depth_im = image.depth();

    // Minimum for object points is 3mm from the plane.
    const float min_plane_dist = 0.003;

    foreach_idx(i, object_points)
    {
        Point3f pobj = object_points[i];
        Point3f crd = toGridWorld(pobj);
        if (!is_zyx_in_range(m_voxels, crd.z, crd.y, crd.x))
            continue;
        if (m_voxels(crd.z, crd.y, crd.x) == Object)
            continue;
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
            if (m_voxels(crd.z, crd.y, crd.x) != Background)
            {
                m_voxels(crd.z, crd.y, crd.x) = Object;
                m_voxels_color(crd.z, crd.y, crd.x) = bgr_to_rgb(color);
            }
        }
    }   

    // Second loop to apply to true color on originally detected points.
    foreach_idx(i, object_points)
    {
        Point3f pobj = object_points[i];

        Point3f rgb_p = rgb_pose.projectToImage(pobj);
        Vec3b color (255,255,255);
        if (!is_yx_in_range(image.rgb(), rgb_p.y, rgb_p.x))
            continue;

        color = image.rgb()(rgb_p.y, rgb_p.x);

        Point3f crd = toGridWorld(pobj);
        if (!is_zyx_in_range(m_voxels, crd.z, crd.y, crd.x))
            continue;

        m_voxels(crd.z, crd.y, crd.x) = Object;
        m_voxels_color(crd.z, crd.y, crd.x) = bgr_to_rgb(color);
    }

    closeVolume();
    // openVolume();

    // Remove background voxels.
    for_all_drc(m_voxels)
    {
        if (m_voxels(d, r, c) != Object)
            continue;

        cv::Point3f p3d = toRealWorld(Point3f(c,r,d));
        Point3f p = depth_pose.projectToImage(p3d);

        if (!is_yx_in_range(depth_im, p.y, p.x))
            continue;

        // No valid depth measurement.
        if (depth_im(p.y, p.x) < 1e-5)
            m_voxels(d,r,c) = Unknown;

        // Depth value is further than voxel, eliminate voxel.
        if (depth_im(p.y, p.x) > p.z)
        {
            m_voxels(d,r,c) = Background;
        }
    }
    return true;
}

void TableObjectRGBDModeler :: fillDepthImage(RGBDImage& painted_image,
                                              const RGBDImage& image,
                                              const Pose3D& depth_pose)
{
    image.copyTo(painted_image);
    Mat1f& depth_im = painted_image.depthRef();
    Mat1b& depth_mask = painted_image.depthMaskRef();

    // FIXME: this is overkill to determine the bounding box!
    std::vector<cv::Point3f> object_pixels;
    for_all_drc(m_voxels)
    {
        if (m_voxels(d, r, c) != Object)
            continue;

        cv::Point3f p3d = toRealWorld(Point3f(c,r,d));
        Point3f p = depth_pose.projectToImage(p3d);

        if (!is_yx_in_range(depth_im, p.y, p.x))
            continue;

        // No valid depth measurement.
        if (depth_im(p.y, p.x) < 1e-5)
            continue;

        object_pixels.push_back(p);
    }

    Rect3f bbox = bounding_box(object_pixels);
    int border_x = bbox.width * 0.5;
    int border_y = bbox.height * 0.5;
    bbox.x -= border_x/2;
    bbox.y -= border_y/2;
    bbox.width += border_x;
    bbox.height += border_y;
    Rect r(bbox.x, bbox.y, bbox.width, bbox.height);
    r = r & Rect(0,0,depth_im.cols, depth_im.rows);

    Mat1b object_img(r.height, r.width);
    Mat1f object_depth(r.height, r.width);
    Mat3b object_color(r.height, r.width);

    depth_im(r).copyTo(object_depth);
    painted_image.mappedRgb()(r).copyTo(object_color);

    // imwrite("/tmp/debug_color.png", object_color);

    computeMesh();
    computeSurfaceMesh();
    MeshRenderer renderer(depth_im.cols, depth_im.rows);
    renderer.setMesh(m_mesh);
    renderer.setPose(depth_pose);
    cv::Mat4b render_image(depth_im.size());
    render_image = Vec4b(0,0,0,0);
    renderer.renderToImage(render_image, MeshRenderer::NORMAL);

    const uchar background_voxel= 0;
    const uchar unknown_voxel = 1;
    const uchar object_voxel = 2;
    object_img = background_voxel;

    for_all_rc(render_image)
    {
        if (cv::norm(render_image(r,c)) > 1e-1)
        {
            int x = c-bbox.x;
            int y = r-bbox.y;
            object_img(y,x) = object_voxel;
        }
    }

    for_all_rc(object_img)
    {
        if (object_depth(r,c) < 1e-5)
            object_img(r,c) = unknown_voxel;
    }

    // imwrite_normalized("/tmp/debug_object.png", object_img);

    HSColorModel object_model;
    HSColorModel background_model;
    Mat1b mask (object_img.size());

    for_all_rc(mask) mask(r,c) = (object_img(r,c) == object_voxel) ? 255 : 0;
    {
        cv::Mat1b tmp;
        cv::morphologyEx(mask, tmp,
                         cv::MORPH_ERODE,
                         getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(5,5)));
        tmp.copyTo(mask);
    }
    object_model.build(object_color, mask);

    for_all_rc(mask) mask(r,c) = (object_img(r,c) == background_voxel) ? 255 : 0;
    background_model.build(object_color, mask);

    cv::Mat3b debug;
    object_model.show(debug);
    // imwrite("/tmp/debug_color_model.png", debug);

    background_model.show(debug);
    // imwrite("/tmp/debug_background_model.png", debug);

    cv::Mat1f object_likelihood, background_likelihood;
    object_model.backProject(object_color, object_likelihood);
    background_model.backProject(object_color, background_likelihood);

    // imwrite_normalized("/tmp/debug_object_likelihood.png", object_likelihood);
    // imwrite_normalized("/tmp/debug_background_likelihood.png", background_likelihood);

    for_all_rc(object_img)
    {
        if (object_img(r,c) != unknown_voxel) continue;

        if (object_likelihood(r,c) > background_likelihood(r,c))
            object_img(r,c) = object_voxel;
        else
            object_img(r,c) = background_voxel;
    }

    {
        cv::Mat1b tmp;
        cv::morphologyEx(object_img, tmp,
                         cv::MORPH_OPEN,
                         getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(3,3)));
        cv::morphologyEx(tmp, object_img,
                         cv::MORPH_CLOSE,
                         getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(7,7)));
    }
    // imwrite_normalized("/tmp/debug_object_final.png", object_img);

    // imwrite_normalized("/tmp/debug_depth.png", object_depth);
    double min_depth = 0, max_depth = 0;
    minMaxLoc(object_depth, &min_depth, &max_depth);

    cv::Mat1b depth1b(object_depth.size());
    for_all_rc(object_depth)
    {
        depth1b(r,c) = ntk::math::floor(object_depth(r,c)*255.0 / max_depth);
    }

    cv::Mat1b paint_mask(object_img.size());
    paint_mask = 0u;
    for_all_rc(paint_mask)
    {
        if (!object_img(r,c) || object_depth(r,c) < 1e-5)
            paint_mask(r,c) = 255;
    }
    {
        cv::Mat1b tmp;
        cv::morphologyEx(paint_mask, tmp,
                         cv::MORPH_DILATE,
                         getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(10,10)));
        tmp.copyTo(paint_mask);
    }
    // imwrite("/tmp/debug_paint_mask.png", paint_mask);

    cv::Mat1b painted_depth1b;
    cv::inpaint(depth1b, paint_mask, painted_depth1b, 10, INPAINT_TELEA);

    // imwrite("/tmp/debug_painted_depth1b.png", painted_depth1b);

    for_all_rc(depth1b)
    {
        if (object_img(r,c) != object_voxel) continue;
        object_depth(r,c) = painted_depth1b(r,c) / 255.0 * max_depth;
    }
    // imwrite_normalized("/tmp/debug_depth_painted.png", object_depth);

    for_all_rc(object_depth)
    {
        int mr = r+bbox.y;
        int mc = c+bbox.x;
        if (depth_im(mr,mc) < 1e-5
            && object_img(r, c) == object_voxel
            && object_depth(r, c) > 1e-5)
        {
            depth_im(mr,mc) = object_depth(r, c);
            depth_mask(mr,mc) = 1;
        }
    }
    // imwrite_normalized("/tmp/debug_depth_final.png", depth_im);
}

bool TableObjectRGBDModeler :: addNewView(const RGBDImage& image, Pose3D& depth_pose)
{
    image.copyTo(m_last_image);

    ntk::TimeCount tc("addNewView");

    PointCloud<PointXYZIndex> cloud;
    rgbdImageToPointCloud(cloud, image, depth_pose);
    tc.elapsedMsecs("toPointCloud");

    bool ok = buildVoxelsFromNewView(cloud, image, depth_pose);
    if (!ok)
        return false;
    tc.elapsedMsecs(" -- first Build from Voxels");

    if (m_depth_filling)
    {
        RGBDImage painted_image;
        fillDepthImage(painted_image, image, depth_pose);
        rgbdImageToPointCloud(cloud, painted_image, depth_pose);
        buildVoxelsFromNewView(cloud, painted_image, depth_pose);
    }

    tc.elapsedMsecs(" -- Depth Filling");

    if (m_remove_small_structures)
    {
        closeVolume();
        rankOpenFilter(2);
        // openVolume();
    }

    tc.elapsedMsecs(" -- Remove small structures");

    computeMesh();
    computeSurfaceMesh();
    tc.stop("mesh computed");
    return true;
}

void TableObjectRGBDModeler :: reset()
{
    for_all_drc(m_voxels)
        m_voxels(d,r,c) = Unknown;
    RGBDModeler::reset();
    m_first_view = true;
}

} // ntk
