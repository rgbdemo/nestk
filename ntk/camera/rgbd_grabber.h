/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef NTK_CAMERA_RGBD_GRABBER_H
#define NTK_CAMERA_RGBD_GRABBER_H

#include <ntk/core.h>
#include <ntk/thread/utils.h>
#include <ntk/camera/calibration.h>
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
      m_should_exit(0),
      m_last_frame_tick(0),
      m_framerate(0),
      m_frame_count(0)
  {
    setSynchronous(false);
  }

public:
  /*! Tell the grabber thread to stop grabbing. */
  virtual void setShouldExit() { m_should_exit = true; }

  /*! Set the integration time for Time-of-Flight cameras. */
  virtual void setIntegrationTime(double value) {}
  virtual double integrationTime() const { return -1; }

  /*! Set the tilt angle for motorized grabbers such as Kinect. */
  virtual void setTiltAngle(int angle) {}

  /*! Return the current framerate. */
  virtual double frameRate() const { return m_framerate; }

  /*! Set the calibration data that will be included in each image. */
  void setCalibrationData(ntk::RGBDCalibration& data)
  { m_calib_data = &data; m_rgbd_image.setCalibration(&data); }

  ntk::RGBDCalibration* calibrationData()
  { return m_calib_data; }

  const ntk::RGBDCalibration* calibrationData() const
  { return m_calib_data; }

  /*! Thread safe deep copy. */
  void copyImageTo(RGBDImage& image);

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
  void waitForNextFrame(int timeout_msecs = 1000)
  {
    m_condition_lock.lock();
    m_condition.wait(&m_condition_lock, timeout_msecs);
    m_condition_lock.unlock();
  }

protected:
  void advertiseNewFrame();

protected:
  mutable RecursiveQReadWriteLock m_lock;
  mutable QMutex m_condition_lock;
  QWaitCondition m_condition;
  ntk::RGBDCalibration* m_calib_data;
  RGBDImage m_rgbd_image;
  bool m_should_exit;
  uint64 m_last_frame_tick;
  double m_framerate;
  int m_frame_count;
};

} // ntk

#endif // NTK_CAMERA_RGBD_GRABBER_H
