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


#include "pmd_rgb_grabber.h"

#include <ntk/camera/calibration.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

#include <QWriteLocker>
#include <QApplication>
#include <QEvent>

#if NESTK_USE_PMD

using namespace cv;

namespace ntk
{

void PmdRgbGrabber :: run()
{
  RGBDImage current_rgb;
  RGBDImage current_pmd;
  m_rgbd_image.setCalibration(m_calib_data);

  while (!threadShouldExit())
  {
    waitForNewEvent();
    m_pmd_grabber.newEvent(this);
    m_rgb_grabber.newEvent(this);
    m_pmd_grabber.waitForNextFrame();

    m_pmd_grabber.copyImageTo(current_pmd);
    m_rgb_grabber.copyImageTo(current_rgb);

    {
      QWriteLocker locker(&m_lock);
      cv::swap(current_rgb.rawRgbRef(), m_rgbd_image.rawRgbRef());
      cv::swap(current_pmd.rawDepthRef(), m_rgbd_image.rawDepthRef());
      cv::swap(current_pmd.rawIntensityRef(), m_rgbd_image.rawIntensityRef());
      cv::swap(current_pmd.rawAmplitudeRef(), m_rgbd_image.rawAmplitudeRef());
    }

    advertiseNewFrame();
  }
}

} // ntk

#endif
