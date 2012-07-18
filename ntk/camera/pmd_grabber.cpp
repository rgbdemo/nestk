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

#include "pmd_grabber.h"

#include <ntk/ntk.h>
#include <ntk/utils/opencv_utils.h>

// using stringstream constructors.
#include <iostream>
#include <sstream>

using namespace std;

using namespace cv;

namespace ntk
{

PmdGrabber :: PmdGrabber() : m_integration_time(1000)
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

    //setIntegrationTime(m_integration_time);
    //ntk_dbg(0) << "Integration time set.";

    return true;
}

void PmdGrabber :: checkError (int code)
{
    if (code != PMD_OK)
    {
        char err[256];
        pmdGetLastError (m_hnd, err, 256);
        pmdClose (m_hnd);
        ntk_throw_exception(cv::format("PMD Error code=%d, text=%s", code, err));
    }
}

void PmdGrabber :: run()
{
    cv::Mat1f distance(m_image_size);
    cv::Mat1f amplitude(m_image_size);
    cv::Mat1f intensity(m_image_size);
    cv::Mat_<unsigned> flags(m_image_size);

    while (!m_should_exit)
    {
        waitForNewEvent();

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
  //ntk_dbg_print(hasFilterFlag(RGBDProcessor::FlipColorImage), 0);
  for_all_rc(image.amplitudeRef())
  {
    const float amp = image.amplitude()(r,c);
    if (amp > 1)
    {
      image.amplitudeRef()(r,c) = log(amp);
    }
  }
}

} // ntk
