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


#include "pmd_grabber.h"

#include <ntk/ntk.h>
#include <ntk/utils/opencv_utils.h>

// using stringstream constructors.
#include <iostream>
#include <sstream>

#ifdef NESTK_USE_PMDSDK

using namespace std;

using namespace cv;

namespace ntk
{

PmdGrabber :: PmdGrabber() : m_integration_time(2000)
{
}

PmdGrabber :: ~PmdGrabber()
{
    pmdClose (m_hnd);
}

void PmdGrabber :: setIntegrationTime(unsigned usecs)
{
    m_integration_time = usecs;

    QWriteLocker locker(&m_lock);
    checkError(pmdSetIntegrationTime(m_hnd, 0, usecs));
}

void PmdGrabber :: setFrequency(unsigned freq)
{
    m_frequency = freq;
    QWriteLocker locker(&m_lock);
    checkError(pmdSetModulationFrequency(m_hnd, 0, m_frequency));
}

void PmdGrabber :: setOffset(int indexFreq)
{
    std::stringstream command;
    float dOffset;

    dOffset = mapOffsets[indexFreq];
    m_offset = dOffset;
    QWriteLocker locker(&m_lock);
    command<<"SetSoftOffset 0 "<< dOffset; //offset in m
    checkError(pmdSourceCommand(m_hnd, 0, 0, command.str().c_str()));
}

bool PmdGrabber :: connectToDevice()
{
    if (m_connected)
        return true;

    try
    {
        checkError(pmdOpen (&m_hnd, "camboardnano", "", "camboardnanoproc", ""));
        ntk_dbg(0) << "Camera opened.";

        checkError(pmdUpdate(m_hnd));

        PMDDataDescription dd;
        checkError(pmdGetSourceDataDescription (m_hnd, &dd));
        m_image_size.width = dd.img.numColumns;
        m_image_size.height = dd.img.numRows;

        if (dd.subHeaderType != PMD_IMAGE_DATA)
        {
            ntk_dbg(0) << "Source data is not an image!";
            pmdClose (m_hnd);
            return false;
        }
    }
    catch (...)
    {
        pmdClose (m_hnd);
        return false;
    }

    if (!m_calib_data)
        estimateCalibration();

    setIntegrationTime(m_integration_time);
    //ntk_dbg(0) << "Integration time set.";

    m_connected = true;
    return true;
}

void PmdGrabber :: estimateCalibration()
{
    m_calib_data = new RGBDCalibration();

    m_calib_data->setRawRgbSize(cv::Size(m_image_size.width, m_image_size.height));
    m_calib_data->setRgbSize(cv::Size(m_image_size.width, m_image_size.height));
    m_calib_data->raw_depth_size = cv::Size(m_image_size.width, m_image_size.height);
    m_calib_data->depth_size = cv::Size(m_image_size.width, m_image_size.height);

    float fx, fy, cx, cy;
    float k1, k2, k3, p1, p2;

    char pmd_output[256];

    checkError(pmdSourceCommand(m_hnd, pmd_output, 255, "GetSerialNumber"));
    ntk_dbg_print(pmd_output, 1);

    setCameraSerial(pmd_output);

    checkError(pmdSourceCommand(m_hnd, pmd_output, 255, "IsCalibrationDataLoaded"));
    ntk_dbg_print(pmd_output, 1);

    checkError(pmdSourceCommand(m_hnd, pmd_output, 255, "GetLensParameters"));
    ntk_dbg_print(pmd_output, 1);

    std::stringstream ss (pmd_output);
    ss >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2 >> k3;

    m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->rgb_intrinsics);
    m_calib_data->rgb_intrinsics(0,0) = fx;
    m_calib_data->rgb_intrinsics(1,1) = fy;
    m_calib_data->rgb_intrinsics(0,2) = cx;
    m_calib_data->rgb_intrinsics(1,2) = cy;

    m_calib_data->rgb_distortion = cv::Mat1d(1,5);
    m_calib_data->rgb_distortion(0,0) = k1;
    m_calib_data->rgb_distortion(0,1) = k2;
    m_calib_data->rgb_distortion(0,2) = p1;
    m_calib_data->rgb_distortion(0,3) = p2;
    m_calib_data->rgb_distortion(0,4) = k3;
    m_calib_data->zero_rgb_distortion = false;

    m_calib_data->depth_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->depth_intrinsics);
    m_calib_data->depth_intrinsics(0,0) = fx;
    m_calib_data->depth_intrinsics(1,1) = fy;
    m_calib_data->depth_intrinsics(0,2) = cx;
    m_calib_data->depth_intrinsics(1,2) = cy;

    m_calib_data->depth_distortion = cv::Mat1d(1,5);
    m_calib_data->depth_distortion(0,0) = k1;
    m_calib_data->depth_distortion(0,1) = k2;
    m_calib_data->depth_distortion(0,2) = p1;
    m_calib_data->depth_distortion(0,3) = p2;
    m_calib_data->depth_distortion(0,4) = k3;
    m_calib_data->zero_depth_distortion = false;

    cv::initUndistortRectifyMap(m_calib_data->depth_intrinsics,
                                m_calib_data->depth_distortion,
                                cv::Mat(), // R
                                m_calib_data->depth_intrinsics, // newCameraMatrix
                                m_image_size,
                                CV_16SC2,
                                m_calib_data->depth_undistort_map1,
                                m_calib_data->depth_undistort_map2);

    m_calib_data->rgb_undistort_map1 = m_calib_data->depth_undistort_map1;
    m_calib_data->rgb_undistort_map2 = m_calib_data->depth_undistort_map2;

    m_calib_data->R = Mat1d(3,3);
    setIdentity(m_calib_data->R);

    m_calib_data->T = Mat1d(3,1);
    m_calib_data->T = 0.;

    m_calib_data->depth_pose = new Pose3D();
    m_calib_data->depth_pose->setCameraParametersFromOpencv(m_calib_data->depth_intrinsics);

    m_calib_data->rgb_pose = new Pose3D();
    m_calib_data->rgb_pose->toRightCamera(m_calib_data->rgb_intrinsics,
                                          m_calib_data->R,
                                          m_calib_data->T);

    m_calib_data->updatePoses();
    m_calib_data->camera_type = "pmdnano";
}

void PmdGrabber :: checkError (int code)
{
    if (code != PMD_OK)
    {
        char err[256];
        pmdGetLastError (m_hnd, err, 256);
        pmdClose (m_hnd);
        std::cerr << cv::format("PMD Error code=%d, text=%s\n", code, err);
        ntk_throw_exception(cv::format("PMD Error code=%d, text=%s", code, err));
    }
}

void PmdGrabber :: run()
{
    cv::Mat1f distance(m_image_size);
    cv::Mat1f amplitude(m_image_size);
    cv::Mat1f intensity(m_image_size);
    cv::Mat_<unsigned> flags(m_image_size);
    m_rgbd_image.setCameraSerial(cameraSerial());
    m_rgbd_image.setCalibration(m_calib_data);

    while (!threadShouldExit())
    {
        waitForNewEvent(-1); // Use infinite timeout in order to honor sync mode.

        checkError(pmdUpdate(m_hnd));
        // checkError(pmdGetIntensities(m_hnd, intensity[0], sizeof(float)*m_image_size.width*m_image_size.height));
        checkError(pmdGetDistances(m_hnd, distance[0], sizeof(float)*m_image_size.width*m_image_size.height));
        checkError(pmdGetAmplitudes(m_hnd, amplitude[0], sizeof(float)*m_image_size.width*m_image_size.height));
        checkError(pmdGetFlags(m_hnd, flags[0], sizeof(unsigned)*m_image_size.width*m_image_size.height));

        const unsigned* flags_ptr = flags.ptr<unsigned>(0);
        const unsigned* flags_end = flags_ptr + flags.rows*flags.cols;
        float* distance_ptr = distance.ptr<float>(0);
        while (flags_end != flags_ptr)
        {
            if (*flags_ptr & PMD_FLAG_INVALID)
                *distance_ptr = 0.f;
            ++flags_ptr;
            ++distance_ptr;
        }

        {
            QWriteLocker locker(&m_lock);

            flip(distance, m_rgbd_image.rawDepthRef(), 0);
            flip(amplitude, m_rgbd_image.rawAmplitudeRef(), 0);
            // flip(intensity, m_rgbd_image.rawIntensityRef(), 0);
            flip(toMat3b(normalize_toMat1b(amplitude)), m_rgbd_image.rawRgbRef(), 0);
        }

        advertiseNewFrame();
    }
}

void PmdRgbProcessor :: processImage(RGBDImage& image)
{
  RGBDProcessor::processImage(image);
  image.mappedDepthRef() = image.depth();
  image.mappedRgbRef() = image.rgb();
}

} // ntk

#endif
