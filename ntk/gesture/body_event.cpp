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

#include "body_event.h"

#include <ntk/gesture/ni_utils.h>
#include <ntk/utils/time.h>

using namespace cv;
using namespace ntk;

namespace ntk
{

const NtkDebug& operator<<(const NtkDebug& os, const BodyEvent& e)
{
  switch(e.kind)
  {
  case BodyEvent::CircleEvent:
    os << "[Event CIRCLE] " << "radius=" << e.velocity << " ntimes=" << e.angle;
    break;
  case BodyEvent::PushEvent:
    os << "[Event PUSH] " << "vel=" << e.velocity << " angle=" << e.angle;
    break;
  case BodyEvent::SteadyEvent:
    os << "[Event STEADY] " << "vel=" << e.velocity << " angle=" << e.angle;
    break;
  case BodyEvent::SwipeDownEvent:
    os << "[Event SWIPE DOWN] " << "vel=" << e.velocity << " angle=" << e.angle;
    break;
  case BodyEvent::SwipeUpEvent:
    os << "[Event SWIPE UP] " << "vel=" << e.velocity << " angle=" << e.angle;
    break;
  case BodyEvent::SwipeLeftEvent:
    os << "[Event SWIPE LEFT] " << "vel=" << e.velocity << " angle=" << e.angle;
    break;
  case BodyEvent::SwipeRightEvent:
    os << "[Event SWIPE RIGHT] " << "vel=" << e.velocity << " angle=" << e.angle;
    break;
  case BodyEvent::WaveEvent:
    os << "[Event WAVE]";
    break;
  default:
    break;
  }

  return os;
}

} // ntk

namespace
{

void XN_CALLBACK_TYPE BodyEventDetectorSessionStart(const XnPoint3D& ptFocus, void *pUserCxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(pUserCxt);
  detector->sessionStartedCallback();
}

void XN_CALLBACK_TYPE BodyEventDetectorSessionEnd(void *pUserCxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(pUserCxt);
  detector->sessionFinishedCallback();
}

// Circle detector
void XN_CALLBACK_TYPE BodyEventDetectorCircle_Circle(XnFloat fTimes,
                                                     XnBool bConfident,
                                                     const XnVCircle* pCircle,
                                                     void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::CircleEvent, pCircle->fRadius, fTimes);
  detector->sendEvent(event);
}

// Push detector
void XN_CALLBACK_TYPE BodyEventDetectorPush_Pushed(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::PushEvent, fVelocity, fAngle);
  detector->sendEvent(event);
}

// Wave detector
void XN_CALLBACK_TYPE BodyEventDetectorWave_Waved(void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::WaveEvent, 0, 0);
  detector->sendEvent(event);
}

// Steady detector
void XN_CALLBACK_TYPE BodyEventDetectorSteady_Steady(XnUInt32 uid, XnFloat fStdDev, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::SteadyEvent, fStdDev, 0);
  detector->sendEvent(event);
}

// Swipe detector.
static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeUp(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::SwipeUpEvent, fVelocity, fAngle);
  detector->sendEvent(event);
}

static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeDown(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::SwipeDownEvent, fVelocity, fAngle);
  detector->sendEvent(event);
}

static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeLeft(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::SwipeLeftEvent, fVelocity, fAngle);
  detector->sendEvent(event);
}

static void XN_CALLBACK_TYPE BodyEventDetectorSwipe_SwipeRight(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::SwipeRightEvent, fVelocity, fAngle);
  detector->sendEvent(event);
}

}

namespace ntk
{

void BodyEventDetector :: initialize(xn::Context& context, xn::DepthGenerator& depth_generator)
{
  m_context = &context;
  m_depth_generator = &depth_generator;
  m_session_manager = new XnVSessionManager();
  XnStatus rc = m_session_manager->Initialize(m_context, "Click,Wave", "RaiseHand");
  if (rc != XN_STATUS_OK)
  {
    ntk_throw_exception("Could not initialize the Session Manager :" + xnGetStatusString(rc));
  }
  m_session_manager->RegisterSession(this,
                                     &BodyEventDetectorSessionStart,
                                     &BodyEventDetectorSessionEnd);

  m_push_detector = new XnVPushDetector;
  m_push_detector->RegisterPush(this, &BodyEventDetectorPush_Pushed);
  m_session_manager->AddListener(m_push_detector);

  m_wave_detector = new XnVWaveDetector;
  m_wave_detector->RegisterWave(this, &BodyEventDetectorWave_Waved);
  m_session_manager->AddListener(m_wave_detector);

  // FIXME: does not work with ROS.
  // m_steady_detector = new XnVSteadyDetector;
  // FIXME: steady API changed. Needs to guards to select the right function.
  // m_steady_detector->RegisterSteady(this, &BodyEventDetectorSteady_Steady);
  // FIXME: this should be accessible from Lua.
  // m_steady_detector->SetDetectionDuration(1000);
  // m_steady_detector->SetMaximumStdDevForSteady(0.01);
  // obsolete with latest openni. m_steady_detector->SetMaximumVelocity(0.005);
  // m_session_manager->AddListener(m_steady_detector);

  m_circle_detector = new XnVCircleDetector;
  m_circle_detector->RegisterCircle(this, &BodyEventDetectorCircle_Circle);
  m_session_manager->AddListener(m_circle_detector);

  m_swipe_detector = new XnVSwipeDetector(true);
  m_swipe_detector->RegisterSwipeUp(this, &BodyEventDetectorSwipe_SwipeUp);
  m_swipe_detector->RegisterSwipeDown(this, &BodyEventDetectorSwipe_SwipeDown);
  m_swipe_detector->RegisterSwipeLeft(this, &BodyEventDetectorSwipe_SwipeLeft);
  m_swipe_detector->RegisterSwipeRight(this, &BodyEventDetectorSwipe_SwipeRight);
  // FIXME: disabled, using custom detector.
  // m_session_manager->AddListener(m_swipe_detector);

#if 1
  m_point_denoiser = new XnVPointDenoiser();
  m_session_manager->AddListener(m_point_denoiser);
  m_point_denoiser->AddListener(this);
#else
  m_session_manager->AddListener(this);
#endif
}

void BodyEventDetector :: shutDown()
{
    m_context = 0;
    m_depth_generator = 0;
    delete m_session_manager; m_session_manager = 0;
    delete m_push_detector; m_push_detector = 0;
    delete m_swipe_detector; m_swipe_detector = 0;
    delete m_wave_detector; m_wave_detector = 0;
    delete m_circle_detector; m_circle_detector = 0;
    delete m_steady_detector; m_steady_detector = 0;
    delete m_point_denoiser; m_point_denoiser = 0;
}

void BodyEventDetector :: update()
{
  m_session_manager->Update(m_context);
}

void BodyEventDetector :: sessionStartedCallback()
{
  ntk_dbg(1) << "Session started!";
}

void BodyEventDetector :: sessionFinishedCallback()
{
  ntk_dbg(1) << "Session finished :-(";
}

void BodyEventDetector :: sendEvent(const BodyEvent& event)
{
  foreach_idx(i, m_listeners)
  {
    m_listeners[i]->triggerEvent(event);
  }
}

void BodyEventDetector :: OnPrimaryPointCreate(const XnVHandPointContext* pContext, const XnPoint3D& ptSessionStarter)
{
  m_prev_hand_points[0] = toPoint3f(pContext->ptPosition);
  m_prev_hand_points[1] = m_prev_hand_points[0];
  uint64 timestamp = ntk::Time::getMillisecondCounter();
  m_prev_timestamps[0] = timestamp;
  m_prev_timestamps[1] = timestamp;
  m_tracked_user_id = pContext->nUserID;
}

void BodyEventDetector :: OnPrimaryPointUpdate(const XnVHandPointContext* pContext)
{
}

void BodyEventDetector :: OnPointUpdate(const XnVHandPointContext* pContext)
{
  m_tracked_user_id = pContext->nUserID;

  XnPoint3D ni_2d;
  m_depth_generator->ConvertRealWorldToProjective(1, &pContext->ptPosition, &ni_2d);

  uint64 timestamp = ntk::Time::getMillisecondCounter();
  int64 deltat = timestamp - m_prev_timestamps[0];
  if (deltat < 1) deltat = 1;

  int64 deltat_prev = m_prev_timestamps[0] - m_prev_timestamps[1];
  Point3f prev_derivate = (m_prev_hand_points[0]-m_prev_hand_points[1]) * (1.0f / float(deltat_prev));

  Point3f pos_3d = toPoint3f(pContext->ptPosition);

  HandPointUpdate event;
  event.timestamp = timestamp;
  event.pos_3d = pos_3d;
  event.pos_2d = toPoint3f(ni_2d);
  event.derivate_3d = (pos_3d-m_prev_hand_points[0]) * (1.0f / deltat);
  event.velocity = cv::norm(event.derivate_3d);
  event.acceleration = cv::norm(event.derivate_3d - prev_derivate);
  m_prev_hand_points[1] = m_prev_hand_points[0];
  m_prev_hand_points[0] = pos_3d;
  m_prev_timestamps[1] = m_prev_timestamps[0];
  m_prev_timestamps[0] = timestamp;

  {
    QWriteLocker locker(&m_lock);
    m_last_hand_position = event.pos_3d;
    m_last_hand_position_2d = event.pos_2d;
  }

  foreach_idx(i, m_listeners)
  {
    m_listeners[i]->triggerHandPoint(event);
  }
}

}

