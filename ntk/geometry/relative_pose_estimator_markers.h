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

#ifndef NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_MARKERS_H
#define NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_MARKERS_H

#include <ntk/geometry/relative_pose_estimator.h>
#include <ntk/aruco/marker.h>

namespace ntk
{

class RelativePoseEstimatorMarkers : public RelativePoseEstimatorFromImages
{
public:
    RelativePoseEstimatorMarkers() : m_marker_size(-1)
    {}

public:
    void setMarkerSize(float size) { m_marker_size = size; }
    const std::vector<aruco::Marker>& detectedMarkersInSourceImage() const  { return m_source_markers; }

public:
    virtual bool estimateNewPose();

private:
    std::vector<aruco::Marker> m_source_markers;
    float m_marker_size;
};

}

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_MARKERS_H
