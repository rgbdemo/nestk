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


#include "rgbd_calibration.h"

#include <ntk/ntk.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/utils.h>
#include <ntk/mesh/mesh.h>

#include <opencv2/highgui/highgui.hpp>

#include <QDir>

using namespace ntk;
using namespace cv;

namespace ntk
{

RGBDCalibration::RGBDCalibration() :
    zero_rgb_distortion(true),
    zero_depth_distortion(true),
    depth_pose(0),
    rgb_pose(0),
    depth_baseline(7.5e-02),
    depth_offset(1090),
    depth_multiplicative_correction_factor(1.0),
    depth_additive_correction_factor(0.0),
    raw_rgb_size(0,0),
    rgb_size(0,0),
    raw_depth_size(0,0),
    depth_size(0,0),
    raw_depth_unit_in_meters(1.f / 1000.f), // default is mm
    min_depth_in_meters(0.4f),
    max_depth_in_meters(5.0f) // kinect defaults
{
    R_extrinsics = Mat1d(3,3);
    setIdentity(R_extrinsics);
    T_extrinsics = Mat1d(3,1);
    T_extrinsics = 0.0;

    R = Mat1d(3,3);
    setIdentity(R);
    T = Mat1d(3,1);
    T = 0.0;
}

RGBDCalibration :: ~RGBDCalibration()
{
    delete depth_pose;
    delete rgb_pose;
}

void RGBDCalibration::copyTo(RGBDCalibration &rhs) const
{
    rgb_intrinsics.copyTo(rhs.rgb_intrinsics);
    rgb_distortion.copyTo(rhs.rgb_distortion);
    rhs.zero_rgb_distortion = zero_rgb_distortion;
    rhs.zero_depth_distortion = zero_depth_distortion;
    depth_intrinsics.copyTo(rhs.depth_intrinsics);
    depth_distortion.copyTo(rhs.depth_distortion);
    infrared_intrinsics.copyTo(rhs.infrared_intrinsics);
    infrared_distortion.copyTo(rhs.infrared_distortion);
    R_extrinsics.copyTo(rhs.R_extrinsics);
    T_extrinsics.copyTo(rhs.T_extrinsics);
    R.copyTo(rhs.R);
    T.copyTo(rhs.T);

    rhs.depth_pose = new Pose3D;
    *rhs.depth_pose = *depth_pose;
    rhs.rgb_pose = new Pose3D;
    *rhs.rgb_pose = *rgb_pose;

    rgb_undistort_map1.copyTo(rhs.rgb_undistort_map1);
    rgb_undistort_map2.copyTo(rhs.rgb_undistort_map2);

    depth_undistort_map1.copyTo(rhs.depth_undistort_map1);
    depth_undistort_map2.copyTo(rhs.depth_undistort_map2);

    rhs.depth_baseline = depth_baseline;
    rhs.depth_offset = depth_offset;
    rhs.depth_multiplicative_correction_factor = depth_multiplicative_correction_factor;
    rhs.depth_additive_correction_factor = depth_additive_correction_factor;

    rhs.raw_rgb_size = raw_rgb_size;
    rhs.rgb_size = rgb_size;
    rhs.raw_depth_unit_in_meters = raw_depth_unit_in_meters;
    rhs.min_depth_in_meters = min_depth_in_meters;
    rhs.max_depth_in_meters = max_depth_in_meters;

    rhs.raw_depth_size = raw_depth_size;
    rhs.depth_size = depth_size;

    rhs.infrared_size = infrared_size;
}

void RGBDCalibration :: updatePoses()
{
    if (!depth_pose || !rgb_pose)
        return;

    depth_pose->setCameraParametersFromOpencv(depth_intrinsics);
    depth_pose->resetCameraTransform();
    depth_pose->applyTransformBefore(T_extrinsics, R_extrinsics);

    *rgb_pose = *depth_pose;
    rgb_pose->toRightCamera(rgb_intrinsics, R, T);
}

void RGBDCalibration :: loadFromFile(const char* filename)
{
    QFileInfo f (filename);
    ntk_throw_exception_if(!f.exists(), "Could not find calibration file.");
    cv::FileStorage calibration_file (filename, CV_STORAGE_READ);
    readMatrix(calibration_file, "rgb_intrinsics", rgb_intrinsics);
    readMatrix(calibration_file, "rgb_distortion", rgb_distortion);
    zero_rgb_distortion = rgb_distortion(0,0) < 1e-5 ? true : false;
    readMatrix(calibration_file, "depth_intrinsics", depth_intrinsics);
    readMatrix(calibration_file, "depth_distortion", depth_distortion);
    zero_depth_distortion = depth_distortion(0,0) < 1e-5 ? true : false;

    readMatrix(calibration_file, "R", R);
    readMatrix(calibration_file, "T", T);
    cv::Mat1i size_mat;
    readMatrix(calibration_file, "rgb_size", size_mat);
    rgb_size = cv::Size(size_mat(0,0), size_mat(0,1));
    readMatrix(calibration_file, "raw_rgb_size", size_mat);
    raw_rgb_size = cv::Size(size_mat(0,0), size_mat(0,1));
    readMatrix(calibration_file, "depth_size", size_mat);
    depth_size = cv::Size(size_mat(0,0), size_mat(0,1));
    readMatrix(calibration_file, "raw_depth_size", size_mat);
    raw_depth_size = cv::Size(size_mat(0,0), size_mat(0,1));

    try {
        cv::Mat1f raw_depth_unit_in_meters_mat (1,1);
        readMatrix(calibration_file, "raw_depth_unit_in_meters", raw_depth_unit_in_meters_mat);
        this->raw_depth_unit_in_meters = raw_depth_unit_in_meters_mat(0,0);
    }
    catch(...)
    {
        ntk_dbg(1) << "Warning: could not load raw_depth_unit_in_meters";
    }

    try {
        cv::Mat1f min_max_depth (2,1);
        readMatrix(calibration_file, "min_max_depth_in_meters", min_max_depth);
        this->min_depth_in_meters = min_max_depth(0,0);
        this->max_depth_in_meters = min_max_depth(1,0);
    }
    catch(...)
    {
        ntk_dbg(1) << "Warning: could not load min_max_depth_in_meters";
    }

    try {
        readMatrix(calibration_file, "R_extrinsics", R_extrinsics);
        readMatrix(calibration_file, "T_extrinsics", T_extrinsics);
    }
    catch (...)
    {
        ntk_dbg(0) << "Warning: could not load extrinsics (R_extrinsics, T_extrinsics).";
    }

    try
    {
        cv::Mat1i size_mat;
        readMatrix(calibration_file, "infrared_size", size_mat);
        infrared_size = cv::Size(size_mat(0,0), size_mat(0,1));
    }
    catch (...)
    {
        ntk_dbg(1) << "Could not read infrared image size, setting default 1280x1024";
        infrared_size = cv::Size(1280, 1024);
    }

    try
    {
        readMatrix(calibration_file, "infrared_intrinsics", infrared_intrinsics);
        readMatrix(calibration_file, "infrared_distortion", infrared_distortion);
    } catch (...)
    {
        ntk_dbg(1) << "Warning: cannot read infrared camera intrinsics, computing from depth.";
        computeInfraredIntrinsicsFromDepth();
    }

    cv::Mat1f depth_calib (1,2);
    try {
        readMatrix(calibration_file, "depth_base_and_offset", depth_calib);
        depth_baseline = depth_calib(0,0);
        depth_offset = depth_calib(0,1);
    }
    catch(...)
    {
        ntk_dbg(1) << "Warning: could not load depth offset";
    }

    try {
        cv::Mat1f depth_multiplicative_correction_factor (1,1);
        readMatrix(calibration_file, "depth_multiplicative_correction_factor", depth_multiplicative_correction_factor);
        this->depth_multiplicative_correction_factor = depth_multiplicative_correction_factor(0,0);
    }
    catch(...)
    {
        ntk_dbg(1) << "Warning: could not load depth multiplicative factor";
    }

    try {
        cv::Mat1f depth_additive_correction_factor (1,1);
        readMatrix(calibration_file, "depth_additive_correction_factor", depth_additive_correction_factor);
        this->depth_additive_correction_factor = depth_additive_correction_factor(0,0);
    }
    catch(...)
    {
        ntk_dbg(1) << "Warning: could not load depth additive correction factor";
    }

    calibration_file.release();

    depth_pose = new Pose3D();
    rgb_pose = new Pose3D();
    updatePoses();
}

void RGBDCalibration :: updateDistortionMaps()
{
    initUndistortRectifyMap(rgb_intrinsics, rgb_distortion,
                            Mat(), rgb_intrinsics, rgb_size, CV_16SC2,
                            rgb_undistort_map1, rgb_undistort_map2);

    initUndistortRectifyMap(depth_intrinsics, depth_distortion,
                            Mat(), depth_intrinsics, depth_size, CV_16SC2,
                            depth_undistort_map1, depth_undistort_map2);
}

void RGBDCalibration :: saveToFile(const char* filename) const
{
    FileStorage output_file (filename,
                             CV_STORAGE_WRITE);
    writeMatrix(output_file, "rgb_intrinsics", rgb_intrinsics);
    writeMatrix(output_file, "rgb_distortion", rgb_distortion);
    writeMatrix(output_file, "depth_intrinsics", depth_intrinsics);
    writeMatrix(output_file, "depth_distortion", depth_distortion);
    if (infrared_intrinsics.data)
        writeMatrix(output_file, "infrared_intrinsics", infrared_intrinsics);
    if (infrared_distortion.data)
        writeMatrix(output_file, "infrared_distortion", infrared_distortion);
    writeMatrix(output_file, "R", R);
    writeMatrix(output_file, "T", T);
    writeMatrix(output_file, "R_extrinsics", R_extrinsics);
    writeMatrix(output_file, "T_extrinsics", T_extrinsics);
    cv::Mat1i size_matrix(1,2);

    size_matrix(0,0) = rgb_size.width;
    size_matrix(0,1) = rgb_size.height;
    writeMatrix(output_file, "rgb_size", size_matrix);

    size_matrix(0,0) = raw_rgb_size.width;
    size_matrix(0,1) = raw_rgb_size.height;
    writeMatrix(output_file, "raw_rgb_size", size_matrix);

    size_matrix(0,0) = depth_size.width;
    size_matrix(0,1) = depth_size.height;
    writeMatrix(output_file, "depth_size", size_matrix);

    size_matrix(0,0) = raw_depth_size.width;
    size_matrix(0,1) = raw_depth_size.height;
    writeMatrix(output_file, "raw_depth_size", size_matrix);

    {
        cv::Mat1f raw_depth_unit_in_meters_mat (1,1);
        raw_depth_unit_in_meters_mat(0,0) = this->raw_depth_unit_in_meters;
        writeMatrix(output_file, "raw_depth_unit_in_meters", raw_depth_unit_in_meters_mat);
    }

    {
        cv::Mat1f min_max_depth (2,1);
        min_max_depth(0,0) = this->min_depth_in_meters;
        min_max_depth(1,0) = this->max_depth_in_meters;
        writeMatrix(output_file, "min_max_depth_in_meters", min_max_depth);
    }

    size_matrix(0,0) = infrared_size.width;
    size_matrix(0,1) = infrared_size.height;
    writeMatrix(output_file, "infrared_size", size_matrix);

    {
        cv::Mat1f depth_calib (1,2);
        depth_calib(0,0) = depth_baseline;
        depth_calib(0,1) = depth_offset;
        writeMatrix(output_file, "depth_base_and_offset", depth_calib);
    }

    {
        cv::Mat1f depth_multiplicative_correction_factor (1,1);
        depth_multiplicative_correction_factor(0,0) = this->depth_multiplicative_correction_factor;
        writeMatrix(output_file, "depth_multiplicative_correction_factor", depth_multiplicative_correction_factor);
    }

    {
        cv::Mat1f depth_additive_correction_factor (1,1);
        depth_additive_correction_factor(0,0) = this->depth_additive_correction_factor;
        writeMatrix(output_file, "depth_additive_correction_factor", depth_additive_correction_factor);
    }

    output_file.release();
}

void RGBDCalibration::computeInfraredIntrinsicsFromDepth()
{
    depth_intrinsics.copyTo(infrared_intrinsics);
    depth_distortion.copyTo(infrared_distortion);

    double& fx = infrared_intrinsics(0,0);
    double& fy = infrared_intrinsics(1,1);
    double& cx = infrared_intrinsics(0,2);
    double& cy = infrared_intrinsics(1,2);

    ntk_dbg(1) << cv::format("fx: %f fy: %f cx: %f cy: %f\n", fx, fy, cx, cy);

    cx = cx - infraredDepthOffsetX();
    cy = cy - infraredDepthOffsetY() + 16;
    double ratio = double(infrared_size.width) / depth_size.width;
    ntk_dbg_print(ratio, 1);
    fx *= ratio;
    fy *= ratio;
    cx *= ratio;
    cy *= ratio;

    ntk_dbg(1) << cv::format("fx: %f fy: %f cx: %f cy: %f\n", fx, fy, cx, cy);
}

void RGBDCalibration::computeDepthIntrinsicsFromInfrared()
{
    infrared_intrinsics.copyTo(depth_intrinsics);
    infrared_distortion.copyTo(depth_distortion);

    double& fx = depth_intrinsics(0,0);
    double& fy = depth_intrinsics(1,1);
    double& cx = depth_intrinsics(0,2);
    double& cy = depth_intrinsics(1,2);

    double ratio = double(infrared_size.width) / depth_size.width;
    fx /= ratio;
    fy /= ratio;
    cx /= ratio;
    cy /= ratio;
    cx = cx + infraredDepthOffsetX();
    cy = cy + infraredDepthOffsetY() - 16;
    // cx = cx;
    // cy = cy - 16;
}

} // ntk
