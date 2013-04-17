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


#include "rgbd_image.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/camera/rgbd_processor.h>

#include <ntk/image/feature.h>

#include <fstream>

#include <QDir>

#if defined(USE_NITE) || defined(NESTK_USE_NITE)
# include <ntk/gesture/skeleton.h>
#else
namespace ntk
{
struct Skeleton
{};

}
#endif

using namespace cv;

namespace ntk
{

RGBDImageHeader::RGBDImageHeader()
    : camera_serial("unknown")
    , timestamp(-1)
    , grabber_type("unknown")
    , calibration (0)
    , filter_min_depth (0.1f)
    , filter_max_depth (10.f)
{
}

void RGBDImageHeader::loadFromDir(const std::string &dir, RGBDCalibrationConstPtr input_calib)
{
    this->directory = dir;

    RGBDCalibrationConstPtr calib;
    if (!input_calib && is_file(dir+"/calibration.yml"))
    {
        // FIXME: use smart pointer.
        RGBDCalibrationPtr new_calib = new RGBDCalibration;
        new_calib->loadFromFile((dir+"/calibration.yml").c_str());
        calib = new_calib;
    }
    else
    {
        calib = input_calib;
    }

    this->calibration = calib;

    if (is_file(dir+"/serial"))
    {
        std::ifstream f ((dir+"/serial").c_str());
        f >> camera_serial;
        f.close ();
    }

    if (is_file(dir+"/grabber-type"))
    {
        std::ifstream f ((dir+"/grabber-type").c_str());
        f >> grabber_type;
        f.close ();
    }

    if (is_file(dir+"/timestamp"))
    {
        std::ifstream f ((dir+"/timestamp").c_str());
        f >> timestamp;
        f.close ();
    }

    if (is_file(dir + "/rgb_pose.avs") && calib)
    {
        ntk::Pose3D pose;
        pose.parseAvsFile((dir + "/rgb_pose.avs").c_str());
        setEstimatedWorldRgbPose(pose);
    }
}

std::string RGBDImageHeader::getUniqueId() const
{
    // FIXME: cache this.
    return cv::format("%s-%08d", camera_serial.c_str(), timestamp);
}

Pose3D RGBDImageHeader :: sensorDepthPose() const
{
    if (0 == calibration)
        return Pose3D();
    return *(calibration->depth_pose);
}

Pose3D RGBDImageHeader :: sensorRgbPose() const
{
    if (0 == calibration)
        return Pose3D();
    Pose3D pose = sensorDepthPose();
    pose.toRightCamera(calibration->rgb_intrinsics, calibration->R, calibration->T);
    return pose;
}

Pose3D RGBDImageHeader :: estimatedWorldRgbPose() const
{
    ntk_assert(calibration, "Calibration must be available!");
    Pose3D pose = estimatedWorldDepthPose();
    if (!pose.isValid())
        return pose;
    pose.toRightCamera(calibration->rgb_intrinsics, calibration->R, calibration->T);
    return pose;
}

void RGBDImageHeader :: setEstimatedWorldRgbPose(const Pose3D& pose)
{
    ntk_assert(calibration, "Calibration must be available!");
    estimated_world_depth_pose = pose;
    estimated_world_depth_pose.toLeftCamera(calibration->depth_intrinsics, calibration->R, calibration->T);
}

RGBDImage::RGBDImage()
    : m_skeleton(0)
{
}

RGBDImage::RGBDImage(const RGBDImage& rhs)
    : EventData(rhs),
      m_skeleton(0)
{
    rhs.copyTo(*this);
}

RGBDImage::RGBDImage(const std::string& dir,
                     RGBDCalibrationConstPtr calib,
                     RGBDProcessor* processor)
    :  m_skeleton(0)
{
    loadFromDir(dir, calib, processor);
}

RGBDImage& RGBDImage::operator =(const RGBDImage &rhs)
{
    rhs.copyTo(*this);
    return *this;
}

RGBDImage :: ~RGBDImage()
{
    if (m_skeleton)
        delete m_skeleton;
}

std::string RGBDImage::getUniqueId() const
{
    // FIXME: cache this.
    return m_header.getUniqueId ();
}

void RGBDImage :: loadFromFile(const std::string& dir,
                               RGBDCalibrationConstPtr calib)
{
    ntk_assert(0, "not implemented.");
}

// Load from a viewXXXX directory.
void RGBDImage :: loadFromDir(const std::string& dir,
                              RGBDCalibrationConstPtr input_calib,
                              RGBDProcessor* processor)
{
    ntk_dbg_print(dir, 2);

    m_header.loadFromDir (dir, input_calib);

    if (!is_file(dir+"/raw/color.png") && is_file(dir+"/color.png"))
    {
        rawRgbRef() = imread(dir + "/color.png", 1);
        rawRgb().copyTo(rgbRef());
        ntk_ensure(rgbRef().data, ("Could not read color image from " + dir).c_str());
    }
    else
    {
        if (is_file(dir + "/raw/color.png"))
        {
            rawRgbRef() = imread(dir + "/raw/color.png", 1);
            ntk_ensure(rawRgbRef().data, ("Could not read raw color image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/color.bmp"))
        {
            rawRgbRef() = imread(dir + "/raw/color.bmp", 1);
            ntk_ensure(rawRgbRef().data, ("Could not read raw color image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/color.jpg"))
        {
            rawRgbRef() = imread(dir + "/raw/color.jpg", 1);
            ntk_ensure(rawRgbRef().data, ("Could not read raw color image from " + dir).c_str());
        }

        if (is_file(dir + "/raw/depth.raw"))
        {
            rawDepthRef() = imread_Mat1f_raw(dir + "/raw/depth.raw");
            ntk_ensure(rawDepthRef().data, ("Could not read raw depth image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/depth.yml"))
        {
            rawDepthRef() = imread_yml(dir + "/raw/depth.yml");
            ntk_ensure(rawDepthRef().data, ("Could not read raw depth image from " + dir).c_str());
        }

        if (is_file(dir + "/raw/depth16bits.raw"))
        {
            rawDepth16bitsRef() = imread_Mat1w_raw(dir + "/raw/depth16bits.raw");
            ntk_ensure(rawDepth16bitsRef().data, ("Could not read raw depth image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/depth16bits.yml"))
        {
            rawDepth16bitsRef() = imread_yml(dir + "/raw/depth16bits.yml");
            ntk_ensure(rawDepth16bitsRef().data, ("Could not read raw depth image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/depth16bits.lzf"))
        {
            rawDepth16bitsRef() = imread_Mat1w_lzf(dir + "/raw/depth16bits.lzf");
            ntk_ensure(rawDepth16bitsRef().data, ("Could not read lzf depth image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/depth16bits.lz4"))
        {
	    ntk_assert (0, "LZ4 parsing is not supported anymore.");
        }

        if (is_file(dir + "/raw/amplitude.raw"))
        {
            rawAmplitudeRef() = imread_Mat1f_raw(dir + "/raw/amplitude.raw");
            ntk_ensure(rawAmplitudeRef().data, ("Could not read raw amplitude image from " + dir).c_str());
        }
        if (is_file(dir + "/raw/amplitude.yml"))
        {
            rawAmplitudeRef() = imread_yml(dir + "/raw/amplitude.yml");
            ntk_ensure(rawAmplitudeRef().data, ("Could not read raw amplitude image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/amplitude.png"))
        {
            rawAmplitudeRef() = imread(dir + "/raw/amplitude.png", 0);
            ntk_ensure(rawAmplitudeRef().data, ("Could not read raw amplitude image from " + dir).c_str());
        }

        if (is_file(dir + "/raw/intensity.raw"))
        {
            rawIntensityRef() = imread_Mat1f_raw(dir + "/raw/intensity.raw");
            ntk_ensure(rawIntensityRef().data, ("Could not read raw intensity image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/intensity.yml"))
        {
            rawIntensityRef() = imread_yml(dir + "/raw/intensity.yml");
            ntk_ensure(rawIntensityRef().data, ("Could not read raw intensity image from " + dir).c_str());
        }
        else if (is_file(dir + "/raw/intensity.png"))
        {
            rawIntensityRef() = imread(dir + "/raw/intensity.png", 0);
            ntk_ensure(rawIntensityRef().data, ("Could not read raw intensity image from " + dir).c_str());
        }
    }

    if (processor)
        processor->processImage(*this);
}

void RGBDImage :: copyTo(RGBDImage& other) const
{
    m_rgb.copyTo(other.m_rgb);
    m_rgb_as_gray.copyTo(other.m_rgb_as_gray);
    m_mapped_rgb.copyTo(other.m_mapped_rgb);
    m_depth.copyTo(other.m_depth);
    m_mapped_depth.copyTo(other.m_mapped_depth);
    m_depth_mask.copyTo(other.m_depth_mask);
    m_mapped_depth_mask.copyTo(other.m_mapped_depth_mask);
    m_depth_to_rgb_coord.copyTo(other.m_depth_to_rgb_coord);
    m_normal.copyTo(other.m_normal);
    m_amplitude.copyTo(other.m_amplitude);
    m_intensity.copyTo(other.m_intensity);
    m_raw_rgb.copyTo(other.m_raw_rgb);
    m_raw_intensity.copyTo(other.m_raw_intensity);
    m_raw_amplitude.copyTo(other.m_raw_amplitude);
    m_raw_depth.copyTo(other.m_raw_depth);
    m_raw_depth_16bits.copyTo(other.m_raw_depth_16bits);
    m_user_labels.copyTo(other.m_user_labels);
    other.m_header = m_header;
#if defined(USE_NITE) || defined(NESTK_USE_NITE)
    if (m_skeleton)
    {
        if (!other.m_skeleton)
            other.m_skeleton = new Skeleton();
        m_skeleton->copyTo(*(other.m_skeleton));
    }
    else
    {
        if (other.m_skeleton)
        {
            delete other.m_skeleton;
            other.m_skeleton = 0;
        }
    }
#endif
}

void RGBDImage :: swap(RGBDImage& other)
{
    cv::swap(m_rgb, other.m_rgb);
    cv::swap(m_rgb_as_gray, other.m_rgb_as_gray);
    cv::swap(m_mapped_rgb, other.m_mapped_rgb);
    cv::swap(m_depth, other.m_depth);
    cv::swap(m_mapped_depth, other.m_mapped_depth);
    cv::swap(m_depth_mask, other.m_depth_mask);
    cv::swap(m_mapped_depth_mask, other.m_mapped_depth_mask);
    cv::swap(m_depth_to_rgb_coord, other.m_depth_to_rgb_coord);
    cv::swap(m_normal, other.m_normal);
    cv::swap(m_amplitude, other.m_amplitude);
    cv::swap(m_intensity, other.m_intensity);
    cv::swap(m_raw_rgb, other.m_raw_rgb);
    cv::swap(m_raw_intensity, other.m_raw_intensity);
    cv::swap(m_raw_amplitude, other.m_raw_amplitude);
    cv::swap(m_raw_depth, other.m_raw_depth);
    cv::swap(m_raw_depth_16bits, other.m_raw_depth_16bits);
    cv::swap(m_user_labels, other.m_user_labels);
    std::swap(m_header, other.m_header);
}

void RGBDImage :: fillRgbFromUserLabels(cv::Mat3b& img) const
{
    if (!m_user_labels.data)
        return;

    const Vec3b colors[] = {
        Vec3b(255,0,0),
        Vec3b(255,255,0),
        Vec3b(255,0,255),
        Vec3b(255,255,255),
        Vec3b(0,255,0),
        Vec3b(0,255,255),
        Vec3b(0,0,255),
    };
    int nb_colors = sizeof(colors) / sizeof(Vec3b);

    img.create(m_user_labels.size());
    for_all_rc(m_user_labels)
    {
        int label = m_user_labels(r,c);
        if (label == 0)
            img(r,c) = Vec3b(0,0,0);
        else
            img(r,c) = colors[label%nb_colors];
    }
}

bool RGBDImage::hasEmptyRawDepthImage() const
{
    for_all_rc(rawDepth())
    {
        if (rawDepth()(r,c) > 1e-5)
            return false;
    }
    return true;
}

void RGBDImage::setFeatures(const ntk::Ptr<FeatureSet>& new_features)
{
    m_features = new_features;
}

} // ntk
