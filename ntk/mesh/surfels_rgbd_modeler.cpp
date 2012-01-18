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

#include "surfels_rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>
#include <ntk/camera/rgbd_processor.h>

using namespace cv;

namespace ntk
{

bool SurfelsRGBDModeler :: mergeToLeftSurfel(Surfel& dest, const Surfel& src)
{
    dest.location = (dest.location*float(dest.n_views) + src.location*float(src.n_views))
            * (1.0f/(dest.n_views+src.n_views));

    cv::Vec3f src_color = src.color;
    cv::Vec3f dest_color = dest.color;
    dest.color = (dest_color*float(dest.n_views) + src_color*float(src.n_views))
            * (1.0f/(dest.n_views+src.n_views));

    if (src.min_camera_angle < dest.min_camera_angle)
    {
        dest.normal = src.normal;
        dest.min_camera_angle = src.min_camera_angle;
        dest.radius = src.radius;
    }

    dest.n_views = dest.n_views + src.n_views;
    return true;
}

bool SurfelsRGBDModeler :: normalsAreCompatible(const Surfel& lhs, const Surfel& rhs)
{
    float normal_angle = acos(lhs.normal.dot(rhs.normal));
    return (normal_angle < (m_update_max_normal_angle*M_PI/180.0));
}

float SurfelsRGBDModeler :: computeSurfelRadius(float depth, float camera_z, double mean_focal)
{
    camera_z = std::max(camera_z, 0.3f);
    float radius = ntk::math::sqrt1_2 * depth / (mean_focal * camera_z);
    radius = std::max(radius, m_resolution*0.5f);
    return radius;
}

bool SurfelsRGBDModeler :: addNewView(const RGBDImage& image_, Pose3D& depth_pose)
{
    ntk::TimeCount tc("SurfelsRGBDModeler::addNewView", 1);
    const float max_camera_normal_angle = ntk::deg_to_rad(90);

    RGBDImage image;
    image_.copyTo(image);
    if (!image_.normal().data)
    {
        OpenniRGBDProcessor processor;
        processor.computeNormalsPCL(image);
    }

    Pose3D rgb_pose = depth_pose;
    rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics, image.calibration()->R, image.calibration()->T);

    Pose3D world_to_camera_normal_pose;
    world_to_camera_normal_pose.applyTransformBefore(cv::Vec3f(0,0,0), depth_pose.cvEulerRotation());
    Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose;
    camera_to_world_normal_pose.invert();

    const Mat1f& depth_im = image.depth();
    Mat1b covered_pixels (depth_im.size());
    covered_pixels = 0;

    std::list<Surfel> surfels_to_reinsert;

    // Surfel updating.
    for (SurfelMap::iterator next_it = m_surfels.begin(); next_it != m_surfels.end(); )
    {
        SurfelMap::iterator surfel_it = next_it;
        ++next_it;

        Surfel& surfel = surfel_it->second;
        if (!surfel.enabled())
            continue;

        Point3f surfel_2d = depth_pose.projectToImage(surfel.location);
        bool surfel_deleted = false;
        int r = ntk::math::rnd(surfel_2d.y);
        int c = ntk::math::rnd(surfel_2d.x);
        int d = ntk::math::rnd(surfel_2d.z);
        if (!is_yx_in_range(depth_im, r, c)
                || !image.depthMask()(r, c)
                || !image.isValidNormal(r,c))
            continue;

        const float update_max_dist = getCompatibilityDistance(depth_im(r,c));

        Vec3f camera_normal = image.normal()(r, c);
        normalize(camera_normal);

        Vec3f world_normal = camera_to_world_normal_pose.cameraTransform(camera_normal);
        normalize(world_normal);

        Vec3f eyev = camera_eye_vector(depth_pose, r, c);
        double camera_angle = acos(camera_normal.dot(-eyev));

        if (camera_angle > max_camera_normal_angle)
            continue;

        float normal_angle = acos(world_normal.dot(surfel.normal));
        if (normal_angle > (m_update_max_normal_angle*M_PI/180.0))
        {
            // Removal check. If a surfel has a different normal and is closer to the camera
            // than the new scan, remove it.
            if ((-surfel_2d.z) < depth_im(r,c) && surfel.n_views < 3)
            {
                m_surfels.erase(surfel_it);
                surfel_deleted = true;
            }
            else
                covered_pixels(r,c) = 1;
            continue;
        }

        // If existing surfel is far from new depth value:
        // - If existing one had a worst point of view, and was seen only once, remove it.
        // - Otherwise do not include the new one.
        if (std::abs(surfel_2d.z - depth_im(r,c)) > update_max_dist)
        {
            if (surfel.min_camera_angle > camera_angle && surfel.n_views < 3)
            {
                m_surfels.erase(surfel_it);
                surfel_deleted = true;
            }
            else
                covered_pixels(r,c) = 1;
            continue;
        }

        // Compatible surfel found.
        const float depth = depth_im(r,c) + m_global_depth_offset;

        Point3f p3d = depth_pose.unprojectFromImage(Point2f(c,r), depth);
        cv::Vec3b rgb_color = bgr_to_rgb(image.mappedRgb()(r, c));

        Surfel image_surfel;
        image_surfel.location = p3d;
        image_surfel.normal = world_normal;
        image_surfel.color = rgb_color;
        image_surfel.min_camera_angle = camera_angle;
        image_surfel.n_views = 1;
        image_surfel.radius = computeSurfelRadius(depth, camera_normal[2], depth_pose.meanFocal());
        mergeToLeftSurfel(surfel, image_surfel);

        covered_pixels(r,c) = 1;
        // needs to change the cell?
        Cell new_cell = worldToCell(surfel.location);
        if (new_cell != surfel_it->first)
        {
            surfels_to_reinsert.push_back(surfel);
            m_surfels.erase(surfel_it);
        }
    }

    foreach_const_it(it, surfels_to_reinsert, std::list<Surfel>)
    {
        Cell new_cell = worldToCell(it->location);
        m_surfels.insert(std::make_pair(new_cell, *it));
    }
    tc.elapsedMsecs(" -- updating -- ");

    // imwrite_normalized("debug_covered.png", covered_pixels);
    // imwrite_normalized("debug_depth_mask.png", image.depthMask());

    // Surfel addition
    for (int r = 0; r < depth_im.rows; r += 1)
        for (int c = 0; c < depth_im.cols; c += 1)
        {
            if (!image.depthMask()(r,c) || covered_pixels(r,c) || !image.isValidNormal(r,c))
                continue;
            float depth = depth_im(r,c) + m_global_depth_offset;
            if (depth < 1e-5)
                continue;

            Point3f p3d = depth_pose.unprojectFromImage(Point2f(c,r), depth);
            if (!m_bounding_box.isEmpty() && !m_bounding_box.isPointInside(p3d))
                continue;

            cv::Vec3b rgb_color = bgr_to_rgb(image.mappedRgb()(r,c));

            Vec3f camera_normal = image.normal()(r, c);
            normalize(camera_normal);

            Vec3f world_normal = camera_to_world_normal_pose.cameraTransform(camera_normal);
            normalize(world_normal);

            Vec3f eyev = camera_eye_vector(depth_pose, r, c);
            double camera_angle = acos(camera_normal.dot(-eyev));

            if (camera_angle > max_camera_normal_angle)
                continue;

            Surfel surfel;
            surfel.location = p3d;
            surfel.normal = world_normal;
            // surfel.color = Vec3b(255,0,0);
            surfel.color = rgb_color;
            // int b = 255.*(camera_angle*180./M_PI)/90.;
            // surfel.color = Vec3b(b,b,b);
            surfel.min_camera_angle = camera_angle;
            surfel.radius = computeSurfelRadius(depth, camera_normal[2], depth_pose.meanFocal());

            surfel.n_views = 1;
            Cell cell = worldToCell(surfel.location);
            bool was_updated = false;
            std::pair<SurfelMap::iterator,SurfelMap::iterator> existings = m_surfels.equal_range(cell);
            for (SurfelMap::iterator it = existings.first; !was_updated && it != existings.second; ++it)
            {
                if (normalsAreCompatible(it->second, surfel))
                {
                    mergeToLeftSurfel(it->second, surfel);
                    was_updated = true;
                }
            }
            // FIXME: merge them!
            if (!was_updated)
            {
                m_surfels.insert(std::make_pair(cell, surfel));
            }
        }
    tc.elapsedMsecs(" -- adding new voxels -- ");

    ntk_dbg_print(m_surfels.size(), 1);
    return true;
}

void SurfelsRGBDModeler :: computeMesh()
{
    m_mesh.clear();

    foreach_const_it(it, m_surfels, SurfelMap)
    {
        const Surfel& surfel = it->second;
        if (!surfel.enabled())
            continue;

        if (surfel.n_views < m_min_views)
            continue;

        if (m_use_surfels)
            m_mesh.addSurfel(surfel);
        else
            m_mesh.addPointFromSurfel(surfel);
    }
}

void SurfelsRGBDModeler::setResolution(float r)
{
    if (flt_eq(m_resolution, r, 1e-5))
        return;
    m_resolution = r;
    reset();
}

float ntk::SurfelsRGBDModeler::getCompatibilityDistance(float depth) const
{
    if (depth < 0.8)
        return 0.002;
    else if (depth < 1)
        return 0.003;
    else if (depth < 1.5)
        return 0.005;
    else if (depth < 2)
        return 0.008;
    else if (depth < 3)
        return 0.02;
    else if (depth < 4)
        return 0.04;
    else if (depth < 5)
        return 0.05;
    else if (depth < 6)
        return 0.06;
    return 0.1;
}

}  // ntk
