#ifndef SOFTKINETIC_GRABBER_H
#define SOFTKINETIC_GRABBER_H

#include "rgbd_grabber.h"

#include <ntk/core.h>
#include <ntk/utils/qt_utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/thread/event.h>

#include <DepthSense.hxx>

#include <QThread>

namespace ntk
{

/*!
 * Grab RGB-D images from a Kinect.
 */
class SoftKineticGrabber : public RGBDGrabber
{
public:
  SoftKineticGrabber(int device_id = 0)
    : m_depth_transmitted(0),
      m_rgb_transmitted(0),
      m_device_id(device_id)
  {}

  /*! Connect with the Kinect device. */
  virtual bool connectToDevice();

  /*! Disconnect from the Kinect device. */
  virtual bool disconnectFromDevice();

  virtual std::string grabberType () const { return "softkinetic"; }

  static bool hasDll ();

public:
  void onNewColorSample(DepthSense::ColorNode::NewSampleReceivedData data);
  void onNewDepthSample(DepthSense::DepthNode::NewSampleReceivedData data);
  void onDeviceConnected(DepthSense::Context::DeviceAddedData data);
  void onDeviceDisconnected(DepthSense::Context::DeviceRemovedData data);
  void onNodeConnected(DepthSense::Device::NodeAddedData data);
  void onNodeDisconnected(DepthSense::Device::NodeRemovedData data);
  void configureNode(DepthSense::Node node);

protected:
  void handleNewFrame();
  void estimateCalibration(DepthSense::StereoCameraParameters& parameters);

protected:
  virtual void run();

private:
  RGBDImage m_current_image;
  bool m_depth_transmitted;
  bool m_rgb_transmitted;
  DepthSense::Context m_context;
  DepthSense::DepthNode m_depth_node;
  DepthSense::ColorNode m_color_node;
  int64 m_last_grab_time;
  int m_device_id;
};

} // ntk

#endif // SOFTKINETIC_GRABBER_H
