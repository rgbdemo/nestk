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

#include <fstream>

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
        m_save_calibration(false),
        m_use_compressed_format(true),
        m_include_serial(true),
        m_include_timestamp(true)
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
      std::string path = m_dir.absolutePath().toStdString();
      if (m_include_serial)
          path += "/" + image.cameraSerial();
      path += format("/view", m_frame_index);
      if (m_include_timestamp)
          path += format("-%08d", image.timestamp());
      return path;
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

  void RGBDFrameRecorder :: writeHeader(const RGBDImageHeader& header, const std::string& frame_dir)
  {
      QDir dir (frame_dir.c_str());
      dir.mkpath("raw");

      std::string filename;

      if (!header.camera_serial.empty())
      {
          filename = cv::format("%s/serial", frame_dir.c_str());
          std::ofstream f (filename.c_str());
          f << header.camera_serial;
          f.close ();
      }

      {
          filename = cv::format("%s/timestamp", frame_dir.c_str());
          std::ofstream f (filename.c_str());
          f << header.timestamp;
          f.close ();
      }

      if (!header.grabber_type.empty())
      {
          filename = cv::format("%s/grabber-type", frame_dir.c_str());
          std::ofstream f (filename.c_str());
          f << header.grabber_type;
          f.close ();
      }

      if (m_save_rgb_pose && header.estimatedWorldRgbPose().isValid())
      {
        filename = cv::format("%s/rgb_pose.avs", frame_dir.c_str());
        header.estimatedWorldRgbPose().saveToAvsFile(filename.c_str());
      }

      if (m_save_calibration && header.calibration)
      {
        filename = cv::format("%s/calibration.yml", frame_dir.c_str());
        header.calibration->saveToFile(filename.c_str());
      }
  }

  void RGBDFrameRecorder :: writeFrame(const RGBDImage& image, const std::string& frame_dir)
  {
      std::string raw_frame_dir = format("%s/raw", frame_dir.c_str(), m_frame_index);

      QDir dir (frame_dir.c_str());
      dir.mkpath("raw");

      writeHeader (image.header (), frame_dir);

      std::string filename;

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
          filename = cv::format("%s/raw/color.jpg", frame_dir.c_str());
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

      if (!image.rawDepth16bits().empty())
      {
        if (m_use_binary_raw)
        {
            if (m_use_compressed_format)
            {
                filename = cv::format("%s/raw/depth16bits.lzf", frame_dir.c_str());
                imwrite_Mat1w_lzf(filename.c_str(), image.rawDepth16bits());
            }
            else
            {
                filename = cv::format("%s/raw/depth16bits.raw", frame_dir.c_str());
                imwrite_Mat1w_raw(filename.c_str(), image.rawDepth16bits());
            }
        }
        else
        {
          filename = cv::format("%s/raw/depth16bits.yml", frame_dir.c_str());
          imwrite_yml(filename.c_str(), image.rawDepth16bits());
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
