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

#ifndef NTK_GESTURE_BODYEVENT_H
#define NTK_GESTURE_BODYEVENT_H

#include <ntk/core.h>
#include <ntk/utils/debug.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnVSessionManager.h>
#include <XnVPushDetector.h>
#include <XnVSwipeDetector.h>
#include <XnVWaveDetector.h>
#include <XnVCircleDetector.h>
#include <XnVSteadyDetector.h>
#include <XnVPointControl.h>
#include <XnVPointDenoiser.h>

#include <QReadWriteLock>
#include <QReadLocker>
#include <QWriteLocker>

namespace ntk
{

class BodyEventListener;

class BodyEvent
{
public:
    enum EventKind { InvalidEvent = -1,

                     CircleEvent,
                     PushEvent,
                     PullEvent,
                     SteadyEvent,
                     SwipeUpEvent,
                     SwipeDownEvent,
                     SwipeLeftEvent,
                     SwipeRightEvent,
                     WaveEvent,

                     NumEvents };

public:
  BodyEvent(EventKind kind, float velocity, float angle)
    : kind(kind), velocity(velocity), angle(angle)
  {}

  EventKind kind;
  float velocity;
  float angle;
};
const NtkDebug& operator<<(const NtkDebug& os, const BodyEvent& e);


class BodyEventDetector : public XnVPointControl
{
public:
  BodyEventDetector()
   : m_context(0), m_depth_generator(0), m_tracked_user_id(-1),
     m_last_hand_position(-1,-1,-1), m_last_hand_position_2d(-1,-1,-1)
  {}

  void initialize(xn::Context& m_context, xn::DepthGenerator& depth_generator);
  void update();

  void addListener(BodyEventListener* listener) { m_listeners.push_back(listener); }
  void removeListener(BodyEventListener* listener) { m_listeners.erase(
          std::remove(m_listeners.begin(), m_listeners.end(), listener),
          m_listeners.end()); }

public:
  int getTrackedUserId() const { return m_tracked_user_id; }

  cv::Point3f getLastHandPosition() const
  { QReadLocker locker(&m_lock); return m_last_hand_position; }

  cv::Point3f getLastHandPosition2D() const
  { QReadLocker locker(&m_lock); return m_last_hand_position_2d; }

public:
  void sessionStartedCallback();
  void sessionFinishedCallback();

public:
  void sendEvent(const BodyEvent& event);

public:
  // XnVPointControl
  virtual void OnPointUpdate(const XnVHandPointContext* pContext);
  virtual void OnPrimaryPointUpdate(const XnVHandPointContext* pContext);
  virtual void OnPrimaryPointCreate(const XnVHandPointContext* pContext, const XnPoint3D& ptSessionStarter);

private:
  mutable QReadWriteLock m_lock;
  std::vector<BodyEventListener*> m_listeners;
  xn::Context* m_context;
  xn::DepthGenerator* m_depth_generator;
  XnVSessionManager* m_session_manager;
  XnVPushDetector* m_push_detector;
  XnVSwipeDetector* m_swipe_detector;
  XnVWaveDetector* m_wave_detector;
  XnVCircleDetector* m_circle_detector;
  XnVSteadyDetector* m_steady_detector;
  XnVPointDenoiser* m_point_denoiser;
  cv::Point3f m_prev_hand_points[2];
  uint64 m_prev_timestamps[2];
  int m_tracked_user_id;
  cv::Point3f m_last_hand_position;
  cv::Point3f m_last_hand_position_2d;
};

struct HandPointUpdate
{
    cv::Point3f pos_3d;
    cv::Point3f pos_2d;
    cv::Point3f derivate_3d;
    float velocity;
    float acceleration;
    int64 timestamp;
};

class BodyEventListener
{
public:
  typedef ::ntk::BodyEvent BodyEvent;

public:
  /*!
   * This callback will be called when a gesture event is detected.
   * WARNING: It will be called from the grabber thread.
   */
  virtual void triggerEvent(const BodyEvent& event) {}

  /*!
   * This is callback comes from Nite HandsGenerator.
   * Contrary to the skeleton joint, this does not require calibration.
   * WARNING: It will be called from the grabber thread.
   */
  virtual void triggerHandPoint(const HandPointUpdate& hand_point) {}
};

} // ntk

#endif // NTK_GESTURE_BODYEVENT_H
