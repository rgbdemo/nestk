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

#ifndef NTK_STEREO_CALIBRATION_H
#define NTK_STEREO_CALIBRATION_H

#include <ntk/core.h>
// #include <opencv2/core/core.hpp>
#include <ntk/camera/rgbd_image.h>
#include <ntk/camera/rgbd_processor.h>

#include <QDir>

namespace ntk
{

class MeshGenerator;
class Pose3D;

void crop_image(cv::Mat& image, cv::Size s);

/*! Prepare a chessboard calibration pattern for OpenCV calibrateCamera. */
void calibrationPattern(std::vector< std::vector<cv::Point3f> >& output,
                        int pattern_width,
                        int pattern_height,
                        float square_size,
                        int nb_images);
void calibrationPattern(std::vector<cv::Point3f> & output,
                         int pattern_width,
                         int pattern_height,
                         float square_size);

enum PatternType
{
    PatternChessboard,
    PatternCircles,
    PatternAsymCircles
};

/*! Extract chessboard position using OpenCV. */
void calibrationCorners(const std::string& image_name,
                        const std::string& window_name,
                        int pattern_width, int pattern_height,
                        std::vector<cv::Point2f>& corners,
                        const cv::Mat& image,
                        float scale_factor,
                        PatternType pattern = PatternChessboard,
                        cv::Mat3b *debug_image = 0);

/*!
 * Estimate the 3D transform of a chessboard.
 * You can create a Pose3D using the following code:
 * \code
 *   Pose3D pose;
 *   pose.setCameraParametersFromOpencv(global::depth_intrinsics);
 *   pose.setCameraTransform(H);
 *  \endcode
 */
void estimate_checkerboard_pose(const std::vector<cv::Point3f>& model,
                                const std::vector<cv::Point2f>& img_points,
                                const cv::Mat1d& calib_matrix,
                                cv::Mat1f& H);

double computeCalibrationError(const cv::Mat& F,
                               const std::vector<std::vector<cv::Point2f> >& rgb_corners,
                               const std::vector<std::vector<cv::Point2f> >& depth_corners);

void showCheckerboardCorners(const cv::Mat3b& image,
                             const std::vector<cv::Point2f>& corners,
                             int wait_time = 10);

// Apply translation coeffs, see http://www.ros.org/wiki/kinect_calibration/technical
void kinect_shift_ir_to_depth(cv::Mat3b& im, bool highres = false);

void loadImageList(const QStringList& view_dirs,
                   ntk::RGBDProcessor* processor,
                   ntk::RGBDCalibrationPtr calibration,
                   std::vector<ntk::RGBDImage>& images);

void loadImageList(const QDir& image_dir,
                   const QStringList& view_list,
                   RGBDProcessor *processor,
                   RGBDCalibrationPtr calibration,
                   std::vector<ntk::RGBDImage>& images);

float calibrate_kinect_scale_factor(const std::vector<ntk::RGBDImage>& images,
                                    const std::vector< std::vector<cv::Point2f> >& corners,
                                    int pattern_width, int pattern_height, float pattern_size);

void getCalibratedCheckerboardCorners(const std::vector<RGBDImage>& images,
                                      int pattern_width,
                                      int pattern_height,
                                      PatternType pattern_type,
                                      std::vector< std::vector<cv::Point2f> >& all_corners,
                                      std::vector< std::vector<cv::Point2f> >& good_corners,
                                      bool show_corners = true,
                                      bool use_intensity = false);

void calibrateStereoFromCheckerboard(const std::vector< std::vector<cv::Point2f> >& undistorted_ref_corners,
                                     const std::vector< std::vector<cv::Point2f> >& undistorted_corners,
                                     int pattern_width, int pattern_height, float pattern_size,
                                     ntk::RGBDCalibration &calibration, bool use_intensity = false);

void calibrate_kinect_rgb(const std::vector<ntk::RGBDImage>& images,
                          const std::vector< std::vector<cv::Point2f> >& good_corners,
                          ntk::RGBDCalibration& calibration,
                          int pattern_width,
                          int pattern_height,
                          float pattern_size,
                          ntk::PatternType pattern_type,
                          bool ignore_distortions,
                          bool fix_center,
                          int default_flags = CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO);

void calibrate_kinect_depth_infrared(const std::vector<RGBDImage>& images,
                                     const std::vector< std::vector<cv::Point2f> >& good_corners,
                                     RGBDCalibration& calibration,
                                     int pattern_width,
                                     int pattern_height,
                                     float pattern_size,
                                     ntk::PatternType pattern_type,
                                     bool ignore_distortions,
                                     bool fix_center,
                                     int default_flags = CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_ASPECT_RATIO);

} // ntk

#endif // NTK_STEREO_CALIBRATION_H
