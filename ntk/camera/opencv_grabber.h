#ifndef NTK_OPENCV_GRABBER_H
#define NTK_OPENCV_GRABBER_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_grabber.h>

namespace ntk
{

/*!
 * Grab RGB-D images from an OpenCV camera.
 */
class OpencvGrabber : public RGBDGrabber
{
public:
  OpencvGrabber(const cv::Size& image_size);
  virtual void connectToDevice(int camera_id = 0);

  virtual std::string grabberType () const { return "opencv"; }

public:
  const cv::Size& imageSize() const { return m_image_size; }

protected:
  virtual void run();

private:
  cv::Size m_image_size;
  cv::VideoCapture m_logitech_capture;
};

} // ntk

#endif // NTK_OPENCV_GRABBER_H
