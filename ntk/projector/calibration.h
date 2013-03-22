#ifndef NTK_PROJECTOR_CALIBRATION_H
#define NTK_PROJECTOR_CALIBRATION_H

#include <ntk/core.h>
// #include <opencv2/core/core.hpp>
#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/pose_3d.h>

namespace ntk
{

struct ProjectorCalibration
{
    ProjectorCalibration() :
        pose(0),
        proj_size(1024,768)
    {}

    ~ProjectorCalibration();

    const cv::Size& size() const { return proj_size; }
    void setSize(cv::Size s) { proj_size = s; }

    void loadFromFile(const char* filename);

    // Intrinsics of the projector.
    cv::Mat1d intrinsics;
    cv::Mat1d distortion;

    // Relative pose of the projector. Depth is the reference.
    cv::Mat1d R,T;

    // Pose of the projector.
    Pose3D* pose;

    cv::Size proj_size;

    cv::Mat undistort_map1, undistort_map2;
};

} // ntk

#endif // NTK_PROJECTOR_CALIBRATION_H
