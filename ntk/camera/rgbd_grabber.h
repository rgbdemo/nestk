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

#ifndef NTK_CAMERA_RGBD_GRABBER_H
#define NTK_CAMERA_RGBD_GRABBER_H

#include <ntk/core.h>
#include <ntk/thread/utils.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/thread/event.h>

#include <QThread>

namespace ntk
{

/*!
 * Abstract RGB-D image grabber.
 * The grabber works in its own QT thread.
 */
class RGBDGrabber : public QThread, public EventBroadcaster, public SyncEventListener
{
public:
  RGBDGrabber()
    : m_calib_data(0),
      m_last_frame_tick(0),
      m_framerate(0),
      m_frame_count(0),
      m_connected(false),
      m_camera_serial("unknown"),
      m_initial_timestamp(0),
      m_loop(false),
      m_target_framerate(-1),
      m_should_exit(0)
  {
    setSynchronous(false);
  }

  virtual ~RGBDGrabber() { stop(); }

public:
  /*! Tell the grabber thread to stop grabbing. */
  void setThreadShouldExit (bool should_exit = true);

public:
  /*! Tell whether the grabber should loop when no more data is available. */
  void setLoop(bool loop) { m_loop = loop; }

  /*! Tell whether infrared images should be output. */
  virtual void setIRMode(bool ir) {}

  /*! Set target framerate. */
  virtual void setTargetFrameRate(float rate) { m_target_framerate = rate; }

  /*! Set the integration time for Time-of-Flight cameras. */
  virtual void setIntegrationTime(double value) {}
  virtual double integrationTime() const { return -1; }

  /*! Set the modulation frequency for Time-of-Flight cameras. */
  virtual unsigned frequency() const { return 0; }
  virtual void setFrequency(unsigned freq) {}

  /*! Set an additional distance offset which will be added to all distance values for Time-of-Flight cameras. */
  virtual float offset() const { return 0.0; }
  virtual void setOffset(int indexFreq) {}

  /*! Set the camera serial associated to this device. */
  virtual void setCameraSerial(const std::string& serial) { m_camera_serial = serial; }
  const std::string& cameraSerial() const { return m_camera_serial; }

  /*! Return an identifier of the camera type. */
  virtual std::string grabberType () const = 0;

  /*! Set the tilt angle for motorized grabbers such as Kinect. */
  virtual void setTiltAngle(int angle) {}

  /*! Optimize the sensor for near objects. */
  virtual void setNearMode(bool enable) {}

  /*! Return the current framerate. */
  virtual double frameRate() const { return m_framerate; }

  /*! Set the calibration data that will be included in each image. */
  void setCalibrationData(ntk::RGBDCalibrationPtr data)
  { m_calib_data = data; m_rgbd_image.setCalibration(data); }

  ntk::RGBDCalibrationPtr calibrationData()
  { return m_calib_data; }

  ntk::RGBDCalibrationConstPtr calibrationData() const
  { return m_calib_data; }

  /*! Thread safe deep copy. */
  virtual void copyImageTo(RGBDImage& image);

  /*! Thread safe deep copy of vector of images for multiple grabbers. */
  virtual void copyImagesTo(std::vector<RGBDImage>& images);

  /*!
   * Tell the grabber to wait for notifications before each frame grab.
   * @see SyncEventListener
   */
  virtual void setSynchronous(bool sync)
  { SyncEventListener::setEnabled(sync); }

  /*! Whether the grabber is in synchronous mode. */
  bool isSynchronous() const
  { return SyncEventListener::enabled(); }

  /*! Return true if a frame has already been grabbed. */
  bool hasData() const
  { QReadLocker locker(&m_lock); return m_rgbd_image.depth().data && m_rgbd_image.rgb().data; }

  /*! Returns the last grabbed image. Warning: you should call acquireReadLock first. */
  const RGBDImage& currentImage() const { return m_rgbd_image; }

  /*! Acquire a lock to ensure data do not get modified by the grabber thread. */
  void acquireReadLock() { m_lock.lockForRead(); }

  /*! Release the acquired lock. */
  void releaseReadLock() { m_lock.unlock(); }

  /*! Blocking wait until next frame is ready. */
  void waitForNextFrame(unsigned long timeout_msecs = ULONG_MAX)
  {
    m_condition_lock.lock();
    m_condition.wait(&m_condition_lock, timeout_msecs);
    m_condition_lock.unlock();
  }

  /*! Tell the grabber to stop and wait for thread termination. */
  virtual void stop();

  /*! Obsolete: use connectToDevice. */
  virtual bool initialize() { return connectToDevice(); }

  /*! Connect to the RGBD device. */
  virtual bool connectToDevice() { return true; }

  /*! Disconnect form the RGBD device. */
  virtual bool disconnectFromDevice() { return true; }

  /*! Whether the grabber is connected to the device. */
  bool isConnected() const { return m_connected; }

protected:
  bool threadShouldExit () const;
  void advertiseNewFrame();
  int getCurrentTimestamp();

protected:
  mutable RecursiveQReadWriteLock m_lock;
  mutable QMutex m_condition_lock;
  QWaitCondition m_condition;
  ntk::RGBDCalibrationPtr m_calib_data;
  RGBDImage m_rgbd_image;
  uint64 m_last_frame_tick;
  double m_framerate;
  int m_frame_count;
  bool m_connected;
  std::string m_camera_serial;
  uint64 m_initial_timestamp;
  bool m_loop;
  float m_target_framerate;

private:
  QMutex mutex;
  bool m_should_exit;
};

} // ntk

#endif // NTK_CAMERA_RGBD_GRABBER_H
