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

using namespace cv;
using namespace ntk;

namespace ntk
{

const NtkDebug& operator<<(const NtkDebug& os, const BodyEvent& e)
{
  std::string name = "Unknown.";
  switch(e.kind)
  {
  case BodyEvent::PushEvent:
    os << "[Event PUSH] " << "vel=" << e.velocity << " angle=" << e.angle;
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

// Push detector
void XN_CALLBACK_TYPE BodyEventDetectorPush_Pushed(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  ntk::BodyEventDetector* detector = reinterpret_cast<ntk::BodyEventDetector*>(cxt);
  BodyEvent event(BodyEvent::PushEvent, fVelocity, fAngle);
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

void BodyEventDetector :: initialize(xn::Context& context)
{
  m_context = &context;
  m_session_manager = new XnVSessionManager();
  XnStatus rc = m_session_manager->Initialize(m_context, "Wave", "RaiseHand");
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

  m_swipe_detector = new XnVSwipeDetector(true);
  m_swipe_detector->RegisterSwipeUp(this, &BodyEventDetectorSwipe_SwipeUp);
  m_swipe_detector->RegisterSwipeDown(this, &BodyEventDetectorSwipe_SwipeDown);
  m_swipe_detector->RegisterSwipeLeft(this, &BodyEventDetectorSwipe_SwipeLeft);
  m_swipe_detector->RegisterSwipeRight(this, &BodyEventDetectorSwipe_SwipeRight);
  m_session_manager->AddListener(m_swipe_detector);
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

} // ntk
