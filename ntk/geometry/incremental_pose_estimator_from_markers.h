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

#ifndef NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_FROM_MARKERS_H
#define NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_FROM_MARKERS_H

#include "incremental_pose_estimator.h"
#include <ntk/geometry/relative_pose_estimator_markers.h>
#include <ntk/geometry/plane.h>

namespace ntk
{

class MarkerPose
{
public:
    MarkerPose () : m_id (-1) {}

public:
    void setId(int id) { m_id = id; }
    int id() const { return m_id; }

    void setFromSizeAndCenter(float marker_size, const cv::Point3f& p);
    const cv::Point3f& center() const { return m_center; }
    float markerSize() const { return m_marker_size; }

    const std::vector<cv::Point3f> corners() const { return m_corners; }

private:
    int m_id;
    cv::Point3f m_center;
    float m_marker_size;
    std::vector<cv::Point3f> m_corners;
};

struct MarkerSetup
{
    MarkerSetup(float marker_size = -1) : marker_size(marker_size)
    {}

    float marker_size;
    ntk::Plane ground_plane;
    std::vector<MarkerPose> markers;

    void addMarker(int id, const cv::Point3f& center);
    void shiftToCenter();
};

class AbsolutePoseEstimatorMarkers : public PoseEstimator
{
public:
    AbsolutePoseEstimatorMarkers()
    {}

    void setMarkerSetup(const MarkerSetup& setup)
    { m_marker_setup = setup; }

public:
    void setInputImage(const RGBDImage* image) { m_image = image; }
    virtual bool estimateNewPose();
    const std::vector<aruco::Marker>& detectedMarkers() const { return m_detected_markers; }

private:
    const RGBDImage* m_image;
    MarkerSetup m_marker_setup;
    std::vector<aruco::Marker> m_detected_markers;
};
ntk_ptr_typedefs(AbsolutePoseEstimatorMarkers)

class IncrementalPoseEstimatorFromMarkers : public IncrementalPoseEstimatorFromImage
{
public:
    struct SetupFrame
    {
        RGBDImage image;
        Pose3D pose;
        std::vector<aruco::Marker> markers;
    };

public:
    IncrementalPoseEstimatorFromMarkers()
        : m_started(false),
          m_marker_setup_estimated(false)
    {}

    virtual IncrementalPoseEstimatorFromMarkers* clone() const { return new IncrementalPoseEstimatorFromMarkers(*this); }

public:
    void setMarkerSize(float size);
    void setMarkerSetup(const MarkerSetup& setup);

public:
    virtual bool estimateCurrentPose();
    virtual void reset();
    virtual void drawDetectedMarkers(cv::Mat3b& image);

protected:
    bool estimateMarkerSetup();
    bool isMarkerSetupEstimated() const { return m_marker_setup_estimated; }

private:
    RelativePoseEstimatorMarkers m_relative_pose_estimator;
    AbsolutePoseEstimatorMarkers m_absolute_pose_estimator;
    RGBDImage m_last_image;
    RGBDImage m_first_image;
    Pose3D m_first_pose;
    bool m_started;
    bool m_marker_setup_estimated;
    std::vector<SetupFrame> m_setup_frames;
    std::vector<aruco::Marker> m_last_markers;
    MarkerSetup m_marker_setup;
};
ntk_ptr_typedefs(IncrementalPoseEstimatorFromMarkers)

} // ntk

#endif // NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_FROM_MARKERS_H
