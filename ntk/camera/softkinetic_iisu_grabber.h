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

#ifndef SOFTKINETIC_IISU_GRABBER_H
#define SOFTKINETIC_IISU_GRABBER_H

#include "rgbd_grabber.h"

#include <ntk/core.h>
#include <ntk/utils/qt_utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/thread/event.h>

#include <SDK/iisuSDK.h>

#include <QThread>

namespace ntk
{

/*!
 * Grab RGB-D images from a Kinect.
 */
class SoftKineticIisuGrabber : public RGBDGrabber
{
public:
  SoftKineticIisuGrabber(int device_id = 0)
    : m_depth_transmitted(0),
      m_rgb_transmitted(0),
      m_device_id(device_id),
      m_running(false)
  {}

  /*! Connect with the Kinect device. */
  virtual bool connectToDevice();

  /*! Disconnect from the Kinect device. */
  virtual bool disconnectFromDevice();

  virtual std::string grabberType () const { return "softkinetic"; }

protected:
  void handleNewFrame();
  void estimateCalibration();

protected:
  virtual void run();

protected:
  bool registerData ();
  bool registerEvents ();
  bool registerParameters ();
  void onDataFrame (const SK::DataFrameEvent& event);
  void onError (const SK::ErrorEvent& event);

private:
  RGBDImage m_current_image;
  bool m_depth_transmitted;
  bool m_rgb_transmitted;
  int64 m_last_grab_time;
  int m_device_id;
  bool m_running;

  SK::IisuHandle* m_iisuHandle;
  SK::Device* m_device;
  SK::ParameterHandle<SK::String> m_cameraModel;
  SK::ParameterHandle<int> m_depth_width_parameter;
  SK::ParameterHandle<int> m_depth_height_parameter;
  SK::ParameterHandle<int> m_rgb_width_parameter;
  SK::ParameterHandle<int> m_rgb_height_parameter;
  SK::ParameterHandle<float> m_rgb_hfov;
  SK::ParameterHandle<float> m_rgb_vfov;
  SK::ParameterHandle<float> m_depth_hfov;
  SK::ParameterHandle<float> m_depth_vfov;
  SK::ParameterHandle<bool> m_confidence_filter_parameter;
  SK::ParameterHandle<int> m_confidence_filter_min_threshold;
  SK::ParameterHandle<bool> m_edge_filter_parameter;
  SK::ParameterHandle<bool> m_smooth_filter_parameter;
  SK::DataHandle<SK::Image> m_depth_image;
  SK::DataHandle<SK::Image> m_rgb_image;
};

} // ntk

#endif // SOFTKINETIC_IISU_GRABBER_H
