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


#include "incremental_pose_estimator_from_markers.h"

#include <ntk/aruco/aruco.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/gui/image_window.h>
#include <ntk/hub.h>

bool ntk::IncrementalPoseEstimatorFromMarkers::
estimateCurrentPose()
{
    if (isMarkerSetupEstimated())
    {
        m_absolute_pose_estimator.setInputImage(&m_new_image);
        bool ok = m_absolute_pose_estimator.estimateNewPose();
        if (ok)
            m_current_pose = m_absolute_pose_estimator.estimatedPose();
        m_last_markers = m_absolute_pose_estimator.detectedMarkers();
        return ok;
    }

    // No marker setup defined. Try to build one progressively.
    // First image, try to register it with itself, to check if
    // there are detected markers.
    if (!m_started)
    {
        m_new_image.copyTo(m_last_image);
        m_current_pose = *m_new_image.calibration()->depth_pose;
        m_first_pose = m_current_pose;
        m_new_image.copyTo(m_first_image);
    }

    m_relative_pose_estimator.setSourceImage(m_new_image);
    // m_relative_pose_estimator.setTargetImage(m_last_image);
    // m_relative_pose_estimator.setTargetPose(m_current_pose);
    m_relative_pose_estimator.setTargetImage(m_first_image);
    m_relative_pose_estimator.setTargetPose(m_first_pose);
    bool ok = m_relative_pose_estimator.estimateNewPose();
    if (!ok)
        return false;
    m_last_markers = m_relative_pose_estimator.detectedMarkersInSourceImage();
    m_started = true;

    m_new_image.copyTo(m_last_image);

    SetupFrame frame;
    m_new_image.copyTo(frame.image);
    frame.pose = m_relative_pose_estimator.estimatedSourcePose();
    frame.markers = m_relative_pose_estimator.detectedMarkersInSourceImage();
    // At least two markers are necessary to improve the setup estimation.
    if (frame.markers.size() < 2)
        return false;

    m_setup_frames.push_back(frame);
    if (estimateMarkerSetup())
    {
        m_marker_setup_estimated = true;
    }
    m_current_pose = m_relative_pose_estimator.estimatedSourcePose();
    return true; // FIXME: temp.
}

void ntk::IncrementalPoseEstimatorFromMarkers::
reset()
{
    IncrementalPoseEstimatorFromImage::reset();
    m_started = false;
}

bool ntk::IncrementalPoseEstimatorFromMarkers::
estimateMarkerSetup()
{
    if (m_setup_frames.size() < 10)
        return false;

    return false;
}

void ntk::IncrementalPoseEstimatorFromMarkers::
drawDetectedMarkers(cv::Mat3b &image)
{
    for(size_t i=0; i < m_last_markers.size(); ++i)
    {
        std::cout << m_last_markers[i] << endl;
        m_last_markers[i].draw(image, Scalar(0,0,255), 2);
    }
}

void ntk::IncrementalPoseEstimatorFromMarkers::
setMarkerSize(float size)
{
    m_marker_setup.marker_size = size;
    m_relative_pose_estimator.setMarkerSize(size);
    m_absolute_pose_estimator.setMarkerSetup(m_marker_setup);
}

void ntk::IncrementalPoseEstimatorFromMarkers::
setMarkerSetup(const ntk::MarkerSetup &setup)
{
    m_marker_setup = setup;
    m_absolute_pose_estimator.setMarkerSetup(m_marker_setup);
    m_relative_pose_estimator.setMarkerSize(m_marker_setup.marker_size);
    m_marker_setup_estimated = true;
}

// ============================================================================
bool ntk::AbsolutePoseEstimatorMarkers::
estimateNewPose()
{
    aruco::MarkerDetector detector;

    std::vector<aruco::Marker> markers;

    {
        cv::Mat3b tmp;
        m_image->rgb().copyTo(tmp);
        detector.detect(tmp, markers, *m_image->calibration(), m_marker_setup.marker_size);
        ntk_dbg_print(markers.size(), 2);
        cv::Mat3b debug_im; tmp.copyTo(debug_im);
        for(size_t i=0; i < markers.size(); ++i)
        {
            // std::cout << markers[i] << endl;
            markers[i].draw(debug_im, Scalar(0,0,255), 2);
        }
        // ntk::ImagePublisher::getInstance()->publishImage("markers", debug_im);
        hub::setImage("markers", debug_im);
        // imwrite("/tmp/debug_markers.png", debug_im);
    }
    m_detected_markers = markers;

    // Pairs of markers present in both frames.
    std::vector< std::pair<aruco::Marker, MarkerPose> > marker_pairs;
    foreach_idx(i, markers)
    {
        foreach_idx(j, m_marker_setup.markers)
        {
            if (markers[i].id == m_marker_setup.markers[j].id())
            {
                marker_pairs.push_back(std::make_pair(markers[i], m_marker_setup.markers[j]));
                break;
            }
        }
    }

    // At least two to ensure a good tracking.
    if (marker_pairs.size() < 2)
        return false;

    // Init from first marker pair.
    {
        Pose3D first_marker_pose = marker_pairs[0].first.computePose();
        m_estimated_pose = *m_image->calibration()->depth_pose;
        m_estimated_pose.resetCameraTransform();
        m_estimated_pose.applyTransformBefore(-marker_pairs[0].second.center(), cv::Vec3f(0,0,0));
        m_estimated_pose.applyTransformAfter(first_marker_pose);
    }

    // Refine with all marker pairs.
    {
        std::vector<cv::Point3f> marker_points_3d;
        std::vector<cv::Point3f> image_points;

        foreach_idx(i, marker_pairs)
        {
            for (int k = 0; k < 4; ++k)
            {
                cv::Point3f p3d = marker_pairs[i].second.corners()[k];
                marker_points_3d.push_back(p3d);

                int r = ntk::math::rnd(marker_pairs[i].first[k].y);
                int c = ntk::math::rnd(marker_pairs[i].first[k].x);
                float depth = 0;
                if (is_yx_in_range(m_image->mappedDepth(), r, c))
                    depth = m_image->mappedDepth()(r,c);

                cv::Point3f image_point (marker_pairs[i].first[k].x, marker_pairs[i].first[k].y, depth);
                image_points.push_back(image_point);
            }
        }

        Pose3D delta_target_pose = m_estimated_pose;
        delta_target_pose.toRightCamera(m_image->calibration()->rgb_intrinsics, m_image->calibration()->R, m_image->calibration()->T);
        double error = rms_optimize_3d(delta_target_pose, marker_points_3d, image_points, true /* use_depth */);
        error /= 3; // x, y, z
        error /= marker_pairs.size(); // normalized norm
        ntk_dbg_print(error, 2);
        if (error > 0.01) // 2cm
            return false; // probably a bad estimation.
        delta_target_pose.toLeftCamera(m_image->calibration()->depth_intrinsics, m_image->calibration()->R, m_image->calibration()->T);

        // max 4 cm to be considered as a matching point.
        // double depth_error = rms_optimize_against_depth_image(delta_target_pose, marker_points_3d, m_image->depth(), 0.04);
        // ntk_dbg_print(depth_error, 1);

#if 0
        for (size_t i = 0; i < marker_points_3d.size(); ++i)
        {
            cv::Point3f proj = delta_target_pose.projectToImage(marker_points_3d[i]);
            if (ntk::is_yx_in_range(m_image->depth(), proj.y, proj.x))
            {
                float d = m_image->depth()(proj.y, proj.x);
                if (d < 1e-5)
                    continue;
                ntk_dbg(1) << "----------";
                ntk_dbg_print(proj.z-d, 1);
            }
        }
#endif

        m_estimated_pose = delta_target_pose;
    }

    {
        cv::Mat3b debug_img;
        m_image->rgb().copyTo(debug_img);

        Pose3D rgb_pose = m_estimated_pose;
        rgb_pose.toRightCamera(m_image->calibration()->rgb_intrinsics, m_image->calibration()->R, m_image->calibration()->T);
        for (int i = 0; i < marker_pairs.size(); ++i)
        {
            cv::Point3f center = rgb_pose.projectToImage(marker_pairs[i].second.center());
            cv::putText(debug_img, cv::format("%d", marker_pairs[i].second.id()), cv::Point(center.x, center.y), 0, 1, cv::Scalar(255,0,0));
            for (int k = 0; k < 4; ++k)
            {
                cv::Point3f p3d = marker_pairs[i].second.corners()[k];
                cv::Point3f p = rgb_pose.projectToImage(p3d);
                switch (k)
                {
                case 0:                    
                    cv::circle(debug_img, cv::Point(p.x, p.y), 5, Scalar(0, 0, 255));
                    break;
                case 1:
                    cv::circle(debug_img, cv::Point(p.x, p.y), 5, Scalar(0, 255, 0));
                    break;
                case 2:
                    cv::circle(debug_img, cv::Point(p.x, p.y), 5, Scalar(255, 0, 0));
                    break;
                case 3:
                    cv::circle(debug_img, cv::Point(p.x, p.y), 5, Scalar(255, 255, 255));
                    break;
                default:;
                }
            }
        }
        if (0)
        {
            imshow("debug", debug_img);
            cv::waitKey(0);
        }
        else
        {
            imwrite("/tmp/debug_reprojected.png", debug_img);
        }
    }


    return true;
}

// ============================================================================
void ntk::MarkerSetup::
addMarker(int id, const cv::Point3f& center)
{
    ntk_assert(marker_size > 0, "You must set a marker size before!");

    MarkerPose pose;
    pose.setId(id);
    pose.setFromSizeAndCenter(marker_size, center);
    markers.push_back(pose);
}

void ntk::MarkerSetup::shiftToCenter()
{
    cv::Point3f center (0,0,0);
    for (size_t i = 0; i < markers.size(); ++i)
    {
        center += markers[i].center();
    }
    center *= 1.0f / markers.size();

    for (size_t i = 0; i < markers.size(); ++i)
    {
        markers[i].setFromSizeAndCenter(markers[i].markerSize(), markers[i].center()-center);
    }
}

// ============================================================================
void ntk::MarkerPose::
setFromSizeAndCenter(float marker_size, const cv::Point3f& p)
{
    m_marker_size = marker_size;
    m_center = p;
    m_corners.resize(4);
    m_corners[0] = cv::Point3f(p.x - marker_size/2.0, p.y - marker_size/2.0, p.z);
    m_corners[1] = cv::Point3f(p.x - marker_size/2.0, p.y + marker_size/2.0, p.z);
    m_corners[2] = cv::Point3f(p.x + marker_size/2.0, p.y + marker_size/2.0, p.z);
    m_corners[3] = cv::Point3f(p.x + marker_size/2.0, p.y - marker_size/2.0, p.z);
}
