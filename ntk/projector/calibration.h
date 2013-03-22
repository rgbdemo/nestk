/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 * Copyright (C) 2012 Mariano Tepper <mtepper@dc.uba.ar>
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
 * Author: Mariano Tepper <mtepper@dc.uba.ar>
 */

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
