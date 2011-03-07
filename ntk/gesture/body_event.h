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

namespace ntk
{

class BodyEventListener;

class BodyEvent
{
public:
    enum EventKind { PushEvent = 0,
                     SwipeUpEvent,
                     SwipeDownEvent,
                     SwipeLeftEvent,
                     SwipeRightEvent };

public:
  BodyEvent(EventKind kind, float velocity, float angle)
    : kind(kind), velocity(velocity), angle(angle)
  {}

  EventKind kind;
  float velocity;
  float angle;
};
const NtkDebug& operator<<(const NtkDebug& os, const BodyEvent& e);

class BodyEventDetector
{
public:
  BodyEventDetector()
   : m_context(0)
  {}

  void initialize(xn::Context& m_context);
  void update();
  void addListener(BodyEventListener& listener) { m_listeners.push_back(&listener); }

public:
  void sessionStartedCallback();
  void sessionFinishedCallback();

public:
  void sendEvent(const BodyEvent& event);

private:
  std::vector<BodyEventListener*> m_listeners;
  xn::Context* m_context;
  XnVSessionManager* m_session_manager;
  XnVPushDetector* m_push_detector;
  XnVSwipeDetector* m_swipe_detector;
};

class BodyEventListener
{
public:
  virtual void triggerEvent(const BodyEvent& event) {}
};

} // ntk

#endif // NTK_GESTURE_BODYEVENT_H
