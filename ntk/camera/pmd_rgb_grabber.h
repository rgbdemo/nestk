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

#ifndef NTK_CAMERA_PMD_RGB_GRABBER_H
#define NTK_CAMERA_PMD_RGB_GRABBER_H

#include <ntk/camera/rgbd_grabber.h>

#include <ntk/core.h>
#include <ntk/utils/qt_utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/pmd_grabber.h>

#include <QThread>

#if NESTK_USE_PMD

namespace ntk
{

/*!
 * Provide asychronous stream of RGDB images.
 */
class PmdRgbGrabber : public RGBDGrabber
{
public:
  PmdRgbGrabber(OpencvGrabber& rgb_grabber, PmdGrabber& pmd_grabber)
    : RGBDGrabber(),
      m_rgb_grabber(rgb_grabber),
      m_pmd_grabber(pmd_grabber)
  {}

public:
  OpencvGrabber& rgbGrabber() const { return m_rgb_grabber; }
  PmdGrabber& pmdGrabber() const { return m_pmd_grabber; }

  virtual void setIntegrationTime(double value)
  {
    m_pmd_grabber.setIntegrationTime(value);
  }

  virtual double integrationTime() const { return m_pmd_grabber.integrationTime(); }

  virtual void setFrequency(unsigned int value)
  {
    m_pmd_grabber.setFrequency(value);
  }

  virtual unsigned frequency() const { return m_pmd_grabber.frequency(); }

  virtual void setOffset(int value)
  {
	  m_pmd_grabber.setOffset(value);
  }

  virtual float fOffset() const { return m_pmd_grabber.fOffset(); }

  virtual double frameRate() const
  {
    return m_pmd_grabber.frameRate();
  }

  virtual void setShouldExit()
  {
    RGBDGrabber::setThreadShouldExit();
    m_rgb_grabber.setThreadShouldExit();
    m_pmd_grabber.setThreadShouldExit();
  }

  virtual void setSynchronous(bool sync)
  {
    RGBDGrabber::setSynchronous(sync);
    m_pmd_grabber.setSynchronous(sync);
    m_rgb_grabber.setSynchronous(sync);
  }

protected:
  virtual void run();

private:
  OpencvGrabber& m_rgb_grabber;
  PmdGrabber& m_pmd_grabber;
};

} // ntk

#endif

#endif // NTK_CAMERA_PMD_RGB_GRABBER_H
