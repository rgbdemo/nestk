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

#include "rgbd_frame_recorder.h"

#include <ntk/ntk.h>
#include <ntk/geometry/pose_3d.h>

#include <QDir>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef NESTK_USE_PCL
#include <ntk/mesh/pcl_utils.h>
#include <pcl/io/pcd_io.h>
#endif

using namespace cv;

namespace ntk
{

  RGBDFrameRecorder :: RGBDFrameRecorder(const std::string& directory)
      : m_frame_index(0),
        m_only_raw(true),
        m_use_binary_raw(false),
        m_save_rgb_pose(false),
        m_save_pcl_point_cloud(false),
        m_save_intensity(true),
        m_use_compressed_format(true)
  {
    setDirectory(directory);
  }

  void RGBDFrameRecorder :: setDirectory(const std::string& directory)
  {
    m_dir = QDir(directory.c_str());
    m_frame_index = 0;
  }

  void RGBDFrameRecorder :: setUseBinaryRaw(bool use_it)
  {
    if (QSysInfo::ByteOrder != QSysInfo::LittleEndian)
    {
      ntk_dbg(0) << "WARNING: cannot save images as binary raw, platform is not little endian";
    }
    else
    {
      m_use_binary_raw = use_it;
    }
  }

  std::string RGBDFrameRecorder :: getNextFrameDirectory(const RGBDImage& image) const
  {
    return format("%s/%s/view%04d-%f",
                  m_dir.absolutePath().toStdString().c_str(),
                  image.cameraSerial().c_str(),
                  m_frame_index,
                  image.timestamp());
  }

  void RGBDFrameRecorder :: saveCurrentFrames(const std::vector<RGBDImage>& images)
  {
      for (int image_i = 0; image_i < images.size(); ++image_i)
      {
          std::string frame_dir = getNextFrameDirectory(images[image_i]);
          writeFrame(images[image_i], frame_dir);
      }
      ++m_frame_index;
  }

  void RGBDFrameRecorder :: saveCurrentFrame(const RGBDImage& image)
  {
      std::string frame_dir = getNextFrameDirectory(image);
      writeFrame(image, frame_dir);
      ++m_frame_index;
  }

  void RGBDFrameRecorder :: writeFrame(const RGBDImage& image, const std::string& frame_dir)
  {
      std::string raw_frame_dir = format("%s/raw", frame_dir.c_str(), m_frame_index);

      QDir dir (frame_dir.c_str());
      dir.mkpath("raw");

      std::string filename;

      if (m_save_rgb_pose && image.calibration())
      {
        filename = cv::format("%s/rgb_pose.avs", frame_dir.c_str());
        image.rgbPose().saveToAvsFile(filename.c_str());
      }

      if (!m_only_raw)
      {
        filename = cv::format("%s/color.png", frame_dir.c_str());
        imwrite(filename, image.rgb());
      }

      if (m_save_pcl_point_cloud)
      {
        filename = cv::format("%s/cloud.pcd", frame_dir.c_str());
#ifdef NESTK_USE_PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        rgbdImageToPointCloud(cloud, image);
        pcl::io::savePCDFileASCII(filename.c_str(), cloud);
#endif
      }

      if (m_use_compressed_format)
          filename = cv::format("%s/raw/color.png", frame_dir.c_str());
      else
          filename = cv::format("%s/raw/color.bmp", frame_dir.c_str());
      imwrite(filename, image.rawRgb());

      if (!m_only_raw && image.mappedDepth().data)
      {
        filename = cv::format("%s/mapped_depth.png", frame_dir.c_str());
        imwrite_normalized(filename, image.mappedDepth());

        filename = cv::format("%s/mapped_color.png", frame_dir.c_str());
        imwrite(filename, image.mappedRgb());

        filename = cv::format("%s/depth.yml", frame_dir.c_str());
        imwrite_yml(filename, image.mappedDepth());
      }

      if (!m_only_raw)
      {
        filename = cv::format("%s/raw/depth.png", frame_dir.c_str());
        if (image.rawDepth().data)
          imwrite_normalized(filename.c_str(), image.rawDepth());

        filename = cv::format("%s/depth.png", frame_dir.c_str());
        if (image.depth().data)
          imwrite_normalized(filename.c_str(), image.depth());

        filename = cv::format("%s/intensity.png", frame_dir.c_str());
        if (image.intensity().data)
            imwrite_normalized(filename.c_str(), image.intensity());
      }

      if (image.rawDepth().data)
      {
        if (m_use_binary_raw)
        {
          filename = cv::format("%s/raw/depth.raw", frame_dir.c_str());
          imwrite_Mat1f_raw(filename.c_str(), image.rawDepth());
        }
        else
        {
          filename = cv::format("%s/raw/depth.yml", frame_dir.c_str());
          imwrite_yml(filename.c_str(), image.rawDepth());
        }
      }

      if (m_save_intensity && image.rawIntensity().data)
      {
        if (m_use_binary_raw)
        {
          filename = cv::format("%s/raw/intensity.raw", frame_dir.c_str());
          imwrite_Mat1f_raw(filename.c_str(), image.rawIntensity());
        }
        else
        {
          filename = cv::format("%s/raw/intensity.yml", frame_dir.c_str());
          imwrite_yml(filename.c_str(), image.rawIntensity());
        }
      }

      if (!m_only_raw)
      {
        filename = cv::format("%s/raw/amplitude.png", frame_dir.c_str());
        if (image.rawAmplitude().data)
          imwrite_normalized(filename.c_str(), image.rawAmplitude());

        filename = cv::format("%s/amplitude.png", frame_dir.c_str());
        if (image.amplitude().data)
          imwrite_normalized(filename.c_str(), image.amplitude());
      }

      if (image.rawAmplitude().data)
      {
        if (m_use_binary_raw)
        {
          filename = cv::format("%s/raw/amplitude.raw", frame_dir.c_str());
          imwrite_Mat1f_raw(filename.c_str(), image.rawAmplitude());
        }
        else
        {
          filename = cv::format("%s/raw/amplitude.yml", frame_dir.c_str());
          imwrite_yml(filename.c_str(), image.rawAmplitude());
        }
      }
  }

}
