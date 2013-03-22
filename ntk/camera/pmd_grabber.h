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
