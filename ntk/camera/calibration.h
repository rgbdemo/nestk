/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 * Author: Jorge Garcia Bueno <jgarcia@ing.uc3m.es>, (C) 2010
 */

#ifndef NTK_STEREO_CALIBRATION_H
#define NTK_STEREO_CALIBRATION_H

#include <ntk/core.h>
#include <opencv/cv.h>
#include <ntk/camera/rgbd_image.h>

namespace ntk
{

class MeshGenerator;
class Pose3D;

/*!
 * Store calibration parameters for an RGB+Depth camera.
 * The camera follows a pin-hole models with distortions.
 */
class RGBDCalibration
{
public:
  RGBDCalibration() :
    zero_rgb_distortion(true),
    zero_depth_distortion(true),
    depth_pose(0),
    rgb_pose(0),
    depth_baseline(7.5e-02),
    depth_offset(1090),
    raw_rgb_size(640,480),
    rgb_size(480,480),
    raw_depth_size(204,204),
    depth_size(204,204)
  {}

  ~RGBDCalibration();

  /*! Load calibration parameters from a yaml file. */
  void loadFromFile(const char* filename);

  /*! Save calibration parameters to a yaml file. */
  void saveToFile(const char* filename) const;

  /*! Size of postprocessed rgb images. */
  const cv::Size& rgbSize() const { return rgb_size; }
  void setRgbSize(cv::Size s) { rgb_size = s; }

  /*! Size of raw rgb images. */
  const cv::Size& rawRgbSize() const { return raw_rgb_size; }
  void setRawRgbSize(cv::Size s) { raw_rgb_size = s; }

  /*! Size of the depth image. */
  const cv::Size& depthSize() { return depth_size; }

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

  /*! Whether there are distortions for the depth image or not. */
  bool zero_depth_distortion;

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

  cv::Size raw_rgb_size;
  cv::Size rgb_size;

  cv::Size raw_depth_size;
  cv::Size depth_size;

private:
  RGBDCalibration(const RGBDCalibration& rhs);
};

void crop_image(cv::Mat& image, cv::Size s);

/*! Prepare a chessboard calibration pattern for OpenCV calibrateCamera. */
void calibrationPattern(std::vector< std::vector<cv::Point3f> >& output,
                        int pattern_width,
                        int pattern_height,
                        float square_size,
                        int nb_images);

/*! Extract chessboard position using OpenCV. */
void calibrationCorners(const std::string& image_name,
                        const std::string& window_name,
                        int pattern_width, int pattern_height,
                        std::vector<cv::Point2f>& corners,
                        const cv::Mat& image,
                        float scale_factor);

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

} // ntk

#endif // NTK_STEREO_CALIBRATION_H
