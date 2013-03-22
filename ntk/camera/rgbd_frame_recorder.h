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

#ifndef NTK_CAMERA_FRAMERECORDER_H
#define NTK_CAMERA_FRAMERECORDER_H

#include <ntk/core.h>

#include <ntk/camera/rgbd_grabber.h>

#include <QDir>

namespace ntk
{

/*!
 * Store a serie of RGB-D images in viewXXXX directories.
 */
class RGBDFrameRecorder
{
public:
  /*! Initialize and set the root directory. */
  RGBDFrameRecorder(const std::string& directory);

  /*! Save an image. */
  void saveCurrentFrame(const RGBDImage& image);

  /*! Save a vector of images, automatically appending a suffix. */
  void saveCurrentFrames(const std::vector<RGBDImage>& image);

  /*! Returns the name of directory where the next frame will be written. */
  std::string getNextFrameDirectory(const RGBDImage& image) const;

  /*!
   * Whether the undistorted and postprocessed images should be
   * saved also.
   */
  void setSaveOnlyRaw(bool v) { m_only_raw = v; }

  /*! Whether the calibration file should be saved. */
  void setSaveCalibration(bool v) { m_save_calibration = v; }

  /*! Whether to save a point cloud of the scene. */
  void setSavePCLPointCloud(bool saveit) { m_save_pcl_point_cloud = saveit; }

  /*!
   * Save float images as binary raw. Faster, but only works
   * on Little Endian platforms.
   */
  void setUseBinaryRaw(bool use_it);

  /*! Set the starting index. */
  void setFrameIndex(int index) { m_frame_index = index; }
  void resetFrameIndex() { m_frame_index = 0; }

  /*! Change the parent directory. */
  void setDirectory(const std::string& directory);
  const QDir& directory() const { return m_dir; }

  /*! Set whether an rgb_pose.avs file should be saved along with the image. */
  void setSaveRgbPose(bool saveit) { m_save_rgb_pose = saveit; }

  /*! Set whether the intensity image should be saved if available. */
  void setSaveIntensity(bool saveit) { m_save_intensity = saveit; }

  /*! Whether images should be saved in a compressed format. */
  void setUseCompressedFormat(bool use_compressed) { m_use_compressed_format = use_compressed; }

  /*! Whether images should be saved in a subdirectory with the name of the camera serial. */
  void setIncludeSerial(bool use_it) { m_include_serial = use_it; }

  /*! Whether the timestamp should be included as a suffix. */
  void setIncludeTimestamp(bool use_it) { m_include_timestamp = use_it; }

public:
  void writeFrame(const RGBDImage& image, const std::string& dir);
  void writeHeader(const RGBDImageHeader& header, const std::string& dir);

private:
  QDir m_dir;
  int m_frame_index;
  bool m_only_raw;
  bool m_use_binary_raw;
  bool m_save_rgb_pose;
  bool m_save_pcl_point_cloud;
  bool m_save_intensity;
  bool m_save_calibration;
  bool m_use_compressed_format;
  bool m_include_serial;
  bool m_include_timestamp;
};

} // ntk

#endif // NTK_CAMERA_FRAMERECORDER_H
