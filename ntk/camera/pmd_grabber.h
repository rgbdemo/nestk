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

#ifndef   	NTK_CAMERA_PMD_GRABBER_H_
# define   	NTK_CAMERA_PMD_GRABBER_H_

#include <ntk/core.h>
#include <ntk/camera/rgbd_grabber.h>
#include <map>

#ifdef NESTK_USE_PMDSDK

#include "pmdsdk2.h"

namespace ntk
{

class PmdGrabber : public RGBDGrabber
{
public:
  PmdGrabber();
  bool connectToDevice();

  ~PmdGrabber();

public:
  std::map<int, float> mapOffsets;
  virtual double integrationTime() const { return m_integration_time; }
  virtual void setIntegrationTime(unsigned usecs);
  virtual unsigned frequency() const { return m_frequency; }
  virtual void setFrequency(unsigned freq);
  virtual float fOffset() const { return m_offset; }
  virtual void setOffset(int indexFreq);

  void checkError (int code);

protected:
  virtual void run();
  void estimateCalibration();

private:
  PMDHandle m_hnd;
  unsigned m_integration_time;
  unsigned m_frequency;
  float m_offset;
  cv::Size m_image_size;
};

/*! RGBDProcessor with default parameters for Pmd. */
class PmdRgbProcessor : public RGBDProcessor
{
public:
  PmdRgbProcessor()
    : RGBDProcessor()
  {
    // setFilterFlag(RGBDProcessorFlags::FlipColorImage, true);
    setFilterFlag(RGBDProcessorFlags::FixGeometry, true);
    setFilterFlag(RGBDProcessorFlags::NoAmplitudeIntensityUndistort, true);
  }

  virtual void processImage(RGBDImage& image);
};

}

#endif

#endif // ! NTK_CAMERA_PMD_GRABBER_H_
