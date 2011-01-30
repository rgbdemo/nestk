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
 */

#ifndef NTK_CAMERA_RGBD_IMAGE_H
#define NTK_CAMERA_RGBD_IMAGE_H

#include <ntk/camera/calibration.h>

namespace ntk
{

  class RGBDProcessor;
  class RGBDCalibration;

/*!
 * Stores RGB+Depth data.
 *
 * RGBDImage also stores a pointer on calibration data,
 * and caches postprocessed data such as normals.
 * The rawXXX accessors are the raw data extracted by a grabber.
 * The other accessors refers to postprocessed data, e.g. after
 * RGBDProcessor::processImage .
 */
class CV_EXPORTS RGBDImage
{
public:
  RGBDImage() : m_calibration(0) {}

  /*! Initialize from an viewXXXX directory. */
  RGBDImage(const std::string& dir,
            const RGBDCalibration* calib = 0,
            RGBDProcessor* processor = 0)
  { loadFromDir(dir, calib, processor); }

  /*! Directory path if loaded from disk. */
  const std::string& directory() const { return m_directory; }

  /*! Whether the image was loaded from disk. */
  bool hasDirectory() const { return !m_directory.empty(); }

  /*! Load from a viewXXXX directory. */
  void loadFromDir(const std::string& dir,
                   const RGBDCalibration* calib = 0,
                   RGBDProcessor* processor = 0);

  /*! Load from a single color image. No depth data. */
  void loadFromFile(const std::string& dir,
                    const RGBDCalibration* calib = 0);

  /*! Associated optional calibration data. */
  const RGBDCalibration* calibration() const { return m_calibration; }

  /*! Set an associated calibration data. */
  void setCalibration(const RGBDCalibration* calibration) { m_calibration = calibration; }

  /*! Set an associated viewXXXX directory. */
  void setDirectory(const std::string& dir) { m_directory = dir; }

  /*! Swap content with another image. */
  void swap(RGBDImage& other);

  /*! Deep copy. */
  void copyTo(RGBDImage& other) const;

  /*! Size of the color channel. */
  int rgbWidth() const { return m_rgb.cols; }
  int rgbHeight() const { return m_rgb.rows; }

  /*! Size of the depth channel. */
  int depthWidth() const { return m_depth.cols; }
  int depthHeight() const { return m_depth.rows; }

  /*! Accessors to the postprocessed color channel. */
  cv::Mat3b& rgbRef() { return m_rgb; }
  const cv::Mat3b& rgb() const { return m_rgb; }

  /*! Accessors to the postprocessed depth channel. */
  cv::Mat1f& depthRef() { return m_depth; }
  const cv::Mat1f& depth() const { return m_depth; }

  /*! Accessors to the raw rgb channel. */
  cv::Mat3b& rawRgbRef() { return m_raw_rgb; }
  const cv::Mat3b& rawRgb() const { return m_raw_rgb; }

  /*! Accessors to the raw depth channel. */
  cv::Mat1f& rawDepthRef() { return m_raw_depth; }
  const cv::Mat1f& rawDepth() const { return m_raw_depth; }

  /*! Accessors to the raw intensity channel (ignored with Kinect). */
  cv::Mat1f& rawIntensityRef() { return m_raw_intensity; }
  const cv::Mat1f& rawIntensity() const { return m_raw_intensity; }

  /*! Accessors to the raw depth channel (ignored with Kinect). */
  cv::Mat1f& rawAmplitudeRef() { return m_raw_amplitude; }
  const cv::Mat1f& rawAmplitude() const { return m_raw_amplitude; }

  /*!
   * Accessors to depth mask.
   * This can be used to mark and filter out unreliable depth values.
   */
  cv::Mat1b& depthMaskRef() { return m_depth_mask; }
  const cv::Mat1b& depthMask() const { return m_depth_mask; }

  /*! Accessors to the postprocessed color channel, as a gray image. */
  cv::Mat1b& rgbAsGrayRef() { return m_rgb_as_gray; }
  const cv::Mat1b& rgbAsGray() const { return m_rgb_as_gray; }

  /*!
   * Accessors to the color image aligned with the depth camera.
   * Has the same size as the depth channel.
   */
  cv::Mat3b& mappedRgbRef() { return m_mapped_rgb; }
  const cv::Mat3b& mappedRgb() const { return m_mapped_rgb; }

  /*!
   * Accessors to the depth image aligned with the rgb camera.
   * Has the same size as the rgb channel.
   */
  cv::Mat1f& mappedDepthRef() { return m_mapped_depth; }
  const cv::Mat1f& mappedDepth() const { return m_mapped_depth; }

  /*! Accessors to 3D normals. */
  cv::Mat3f& normalRef() { return m_normal; }
  const cv::Mat3f& normal() const { return m_normal; }

  /*! Accessors to the amplitude image. Ignored with Kinect. */
  cv::Mat1f& amplitudeRef() { return m_amplitude; }
  const cv::Mat1f& amplitude() const { return m_amplitude; }

  /*! Accessors to the intensity image. Ignored with Kinect. */
  cv::Mat1f& intensityRef() { return m_intensity; }
  const cv::Mat1f& intensity() const { return m_intensity; }

  /*! Whether this particular pixel has valid depth information */
  bool pixelHasDepth(int r, int c) const
  { return m_depth_mask.data
        && r < m_depth_mask.rows && c < m_depth_mask.cols && r >= 0 && c >= 0
        && m_depth_mask(r,c);
  }

private:
  cv::Mat3b m_rgb;
  cv::Mat1b m_rgb_as_gray;
  cv::Mat3b m_mapped_rgb;
  cv::Mat1f m_depth;
  cv::Mat1f m_mapped_depth;
  cv::Mat1b m_depth_mask;
  cv::Mat3f m_normal;
  cv::Mat1f m_amplitude;
  cv::Mat1f m_intensity;
  cv::Mat3b m_raw_rgb;
  cv::Mat1f m_raw_intensity;
  cv::Mat1f m_raw_amplitude;
  cv::Mat1f m_raw_depth;
  const RGBDCalibration* m_calibration;
  std::string m_directory;
};

} // ntk

#endif // NTK_CAMERA_RGBD_IMAGE_H
