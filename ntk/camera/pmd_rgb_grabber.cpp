
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
