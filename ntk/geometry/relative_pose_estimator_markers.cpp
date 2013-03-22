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


#include "relative_pose_estimator_markers.h"

#include <ntk/aruco/aruco.h>


bool ntk::RelativePoseEstimatorMarkers::
estimateNewPose()
{
    ntk_assert(m_marker_size > 0, "You must set a marker size before!");

    aruco::MarkerDetector detector;

    std::vector<aruco::Marker> source_markers;
    std::vector<aruco::Marker> target_markers;

    {
        cv::Mat3b tmp;
        m_source_image->rgb().copyTo(tmp);
        detector.detect(tmp, source_markers, *m_source_image->calibration(), m_marker_size);
        cv::Mat3b debug_im; m_source_image->rgb().copyTo(debug_im);
        for(size_t i=0; i < source_markers.size(); ++i)
        {
            std::cout << source_markers[i] << endl;
            source_markers[i].draw(debug_im, Scalar(0,0,255), 2);
        }
        imwrite("/tmp/debug_markers_source.png", debug_im);
    }

    {
        cv::Mat3b tmp;
        m_target_image->rgb().copyTo(tmp);
        detector.detect(tmp, target_markers, *m_target_image->calibration(), m_marker_size);
        cv::Mat3b debug_im; m_target_image->rgb().copyTo(debug_im);
        for(size_t i=0; i < target_markers.size(); ++i)
        {
            std::cout << target_markers[i] << endl;
            target_markers[i].draw(debug_im, Scalar(0,0,255), 2);
        }
        imwrite("/tmp/debug_markers_target.png", debug_im);
    }

    m_source_markers = source_markers;

    // Pairs of markers present in both frames.
    std::vector< std::pair<aruco::Marker, aruco::Marker> > marker_pairs;
    foreach_idx(i, source_markers)
    {
        foreach_idx(j, target_markers)
        {
            if (source_markers[i].id == target_markers[j].id)
            {
                marker_pairs.push_back(std::make_pair(source_markers[i], target_markers[j]));
                break;
            }
        }
    }

    if (marker_pairs.size() < 1)
        return false;

    // Init from first marker pair.
    {
        Pose3D first_source_marker_pose = marker_pairs[0].first.computePose();
        Pose3D inv_first_source_marker_pose = first_source_marker_pose;
        inv_first_source_marker_pose.invert();

        Pose3D first_target_marker_pose = marker_pairs[0].second.computePose();
        Pose3D inv_first_target_marker_pose = first_target_marker_pose;
        inv_first_target_marker_pose.invert();

        m_estimated_pose.resetCameraTransform();
        m_estimated_pose.applyTransformAfter(m_target_pose);
        m_estimated_pose.applyTransformAfter(inv_first_target_marker_pose);
        m_estimated_pose.applyTransformAfter(first_source_marker_pose);
    }

    // Refine with all marker pairs.
    {
        // FIXME: perform a full bundle adjustment.

        std::vector<cv::Point3f> source_marker_points_3d;
        std::vector<cv::Point3f> source_image_points;

        foreach_idx(i, marker_pairs)
        {
            Pose3D source_marker_pose = marker_pairs[i].first.computePose();

            cv::Vec3f markers_3d[4];
            markers_3d[0] = source_marker_pose.cameraTransform(cv::Point3f(-m_marker_size/2.0,  m_marker_size/2.0, 0));
            markers_3d[1] = source_marker_pose.cameraTransform(cv::Point3f(-m_marker_size/2.0, -m_marker_size/2.0, 0));
            markers_3d[2] = source_marker_pose.cameraTransform(cv::Point3f( m_marker_size/2.0, -m_marker_size/2.0, 0));
            markers_3d[3] = source_marker_pose.cameraTransform(cv::Point3f( m_marker_size/2.0,  m_marker_size/2.0, 0));

            for (int k = 0; k < 4; ++k)
            {
                cv::Point3f transformed_marker = m_estimated_pose.invCameraTransform(markers_3d[k]);
                source_marker_points_3d.push_back(transformed_marker);
                float d = m_source_image->depth()(marker_pairs[i].first[k].y, marker_pairs[i].first[k].x);
                cv::Point3f p3d (marker_pairs[i].first[k].x, marker_pairs[i].first[k].y, d);
                p3d = m_estimated_pose.unprojectFromImage(p3d);
                cv::Point3f image_point (marker_pairs[i].second[k].x, marker_pairs[i].second[k].y, 0);
                source_image_points.push_back(image_point);
            }
        }

        Pose3D delta_target_pose = m_target_pose;
        delta_target_pose.toRightCamera(m_target_image->calibration()->rgb_intrinsics, m_target_image->calibration()->R, m_target_image->calibration()->T);
        double error = rms_optimize_3d(delta_target_pose, source_marker_points_3d, source_image_points, true /* use_depth */);
        ntk_dbg_print(error, 2);
        delta_target_pose.invert();
        delta_target_pose.applyTransformBefore(m_target_pose);
        delta_target_pose.toLeftCamera(m_target_image->calibration()->depth_intrinsics, m_target_image->calibration()->R, m_target_image->calibration()->T);
        m_estimated_pose.applyTransformBefore(delta_target_pose);
    }

    {
        cv::Mat3b debug_img;
        m_target_image->rgb().copyTo(debug_img);

        for (int m_i = 0; m_i < marker_pairs.size(); ++m_i)
        {
            for (int i = 0; i < 4; ++i)
            {
                cv::Point2f p = marker_pairs[m_i].first[i];
                cv::Point3f p3d (p.x, p.y, m_source_image->depth()(p.y, p.x));
                p3d = m_estimated_pose.unprojectFromImage(p3d);
                cv::Point3f ptarget = m_target_pose.projectToImage(p3d);
                switch (i)
                {
                case 0:
                    cv::circle(debug_img, cv::Point(ptarget.x, ptarget.y), 5, Scalar(0, 0, 255));
                    break;
                case 1:
                    cv::circle(debug_img, cv::Point(ptarget.x, ptarget.y), 5, Scalar(0, 255, 0));
                    break;
                case 2:
                    cv::circle(debug_img, cv::Point(ptarget.x, ptarget.y), 5, Scalar(255, 0, 0));
                    break;
                case 3:
                    cv::circle(debug_img, cv::Point(ptarget.x, ptarget.y), 5, Scalar(255, 255, 255));
                    break;
                default:;
                }
            }
        }
        if (0)
        {
            imshow("debug", debug_img);
            imshow("source", m_source_image->rgb());
            cv::waitKey(0);
        }
        else
        {
            imwrite("/tmp/debug_reprojected.png", debug_img);
        }
    }


    return true;
}
