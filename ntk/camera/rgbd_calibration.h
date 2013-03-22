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

#pragma once

#include <ntk/core.h>

namespace ntk
{

class Pose3D;

/*!
 * Store calibration parameters for an RGB+Depth camera.
 * The camera follows a pin-hole models with distortions.
 */
class RGBDCalibration
{
public:
    static float infraredDepthOffsetX() { return -3.f; }
    static float infraredDepthOffsetY() { return -3.f; }

public:
  RGBDCalibration();

  ~RGBDCalibration();

  bool isValid () const { return raw_depth_size.width > 0 || raw_rgb_size.width > 0; }

  /*! Deep copy. */
  void copyTo(RGBDCalibration& rhs) const;

  /*! Update depth_pose and rgb_pose from read parameters. */
  void updatePoses();

  /*! Update the distortion maps based on distortion intrinsics. */
  void updateDistortionMaps();

  /*! Load calibration parameters from a yaml file. */
  void loadFromFile(const char* filename);

  /*! Save calibration parameters to a yaml file. */
  void saveToFile(const char* filename) const;

  /*! Size of postprocessed rgb images. */
  const cv::Size& rgbSize() const { return rgb_size; }
  void setRgbSize(cv::Size s) { rgb_size = s; }

  /*! Size of the infrared image. */
  const cv::Size& infraredSize() const { return infrared_size; }

  /*! Size of raw rgb images. */
  const cv::Size& rawRgbSize() const { return raw_rgb_size; }
  void setRawRgbSize(cv::Size s) { raw_rgb_size = s; }

  /*! Size of the depth image. */
  const cv::Size& rawDepthSize() const { return raw_depth_size; }
  const cv::Size& depthSize() const { return depth_size; }

  float rawDepthUnitInMeters () const { return raw_depth_unit_in_meters; }
  void setRawDepthUnitInMeters (float unit) { raw_depth_unit_in_meters = unit; }

  float minDepthInMeters () const { return min_depth_in_meters; }
  float maxDepthInMeters () const { return max_depth_in_meters; }
  void setMinDepthInMeters (float d) { min_depth_in_meters = d; }
  void setMaxDepthInMeters (float d) { max_depth_in_meters = d; }

  /*! Deduce infrared intrinsics from depth intrinsics. */
  void computeInfraredIntrinsicsFromDepth();

  /*! Deduce depth intrinsics from infrared intrinsics. */
  void computeDepthIntrinsicsFromInfrared();

  /*! Intrinsics 3x3 matrix for the rgb channel */
  cv::Mat1d rgb_intrinsics;

  /*! Distortion 1x5 matrix for rgb channel. */
  cv::Mat1d rgb_distortion;

  /*! Whether there are distortions for the rgb image or not. */
  bool zero_rgb_distortion;

  /*! Intrinsics 3x3 matrix for the depth channel */
  cv::Mat1d depth_intrinsics;

  /*! Distortion 1x5 matrix for depth channel. */
  cv::Mat1d depth_distortion;

  /*! Intrinsics 3x3 matrix for the infrared channel */
  cv::Mat1d infrared_intrinsics;

  /*! Distortion 1x5 matrix for infrared channel. */
  cv::Mat1d infrared_distortion;

  /*! Whether there are distortions for the depth image or not. */
  bool zero_depth_distortion;

  /*!
   * Rotation and translation of the Depth sensor.
   * The depth sensor is the reference, so these are camera extrinsics.
   * R is a 3x3 rotation matrix, and T a 1x3 translation vector.
   */
  cv::Mat1d R_extrinsics, T_extrinsics;

  /*!
   * Rotation and translation between RGB and Depth sensor.
   * The depth sensor is the reference.
   * R is a 3x3 rotation matrix, and T a 1x3 translation vector.
   * This is estimated from OpenCV stereoCalibrate.
   */
  cv::Mat1d R,T;

  /*! Pose of the depth camera. @see Pose3D. */
  Pose3D* depth_pose;

  /*! Pose of the rgb camera. @see Pose3D. */
  Pose3D* rgb_pose;

  /*! Distortion maps for cv::undistort. */
  cv::Mat rgb_undistort_map1, rgb_undistort_map2;
  cv::Mat depth_undistort_map1, depth_undistort_map2;

  /*!
   * Depth baseline parameter for Kinect depth computation.
   * See http://www.ros.org/wiki/kinect_calibration/technical .
   */
  double depth_baseline;

  /*!
   * Depth offset parameter for Kinect depth computation.
   * See http://www.ros.org/wiki/kinect_calibration/technical .
   */
  double depth_offset;

  /*! Depth correction factor to multiply with. */
  double depth_multiplicative_correction_factor;

  /*! Depth correction factor to add. */
  double depth_additive_correction_factor;

  cv::Size raw_rgb_size;
  cv::Size rgb_size;

  cv::Size raw_depth_size;
  cv::Size depth_size;

  float raw_depth_unit_in_meters;
  float max_depth_in_meters;
  float min_depth_in_meters;

  cv::Size infrared_size;

private:
  RGBDCalibration(const RGBDCalibration& rhs);
};
ntk_ptr_typedefs (RGBDCalibration)

} // ntk

