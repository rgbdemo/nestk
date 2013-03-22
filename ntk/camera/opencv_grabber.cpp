
#include "opencv_grabber.h"

#include <ntk/ntk.h>

#include <QDir>

using namespace cv;
using namespace ntk;

OpencvGrabber :: OpencvGrabber(const cv::Size& size)
  : m_image_size(size)
{  
}

void OpencvGrabber :: connectToDevice(int camera_id)
{
  if (!m_logitech_capture.open(camera_id))
    ntk_throw_exception("Could not open logitech camera.");

  m_logitech_capture.set(CV_CAP_PROP_FRAME_WIDTH, m_image_size.width);
  m_logitech_capture.set(CV_CAP_PROP_FRAME_HEIGHT, m_image_size.height);
}

void OpencvGrabber :: run()
{
  cv::Mat3b logitech_image (m_image_size);
  m_rgbd_image.rawRgbRef().create(m_image_size);

  while (!threadShouldExit())
  {
    waitForNewEvent(-1); // Use infinite timeout in order to honor sync mode.

    m_logitech_capture >> logitech_image;    

    //ntk_assert(logitech_image.cols == m_image_size.width, "Wrong image size");
    if (logitech_image.data == 0)
      ntk_throw_exception("Error adquiring frame from Logitech. Connected?");

    {
      QWriteLocker locker(&m_lock);
      cv::swap(logitech_image, m_rgbd_image.rawRgbRef());
    }

    advertiseNewFrame();
  }
}
