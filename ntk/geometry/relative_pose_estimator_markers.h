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
