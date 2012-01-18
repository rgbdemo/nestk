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

#include "event.h"
#include <ntk/utils/time.h>

#include <QApplication>

namespace ntk
{

EventData::~EventData()
{
}

EventListener::EventListener()
    : m_last_frame_tick(0),
      m_framerate(0),
      m_frame_count(0)
{

}

void EventListener::reportNewEventProcessed()
{
    ++m_frame_count;
    float tick = ntk::Time::getMillisecondCounter();
    float delta_tick = (tick - m_last_frame_tick);
    if (delta_tick > 1000)
    {
        m_framerate = (1000.f * m_frame_count) / (delta_tick);
        m_last_frame_tick = tick;
        m_frame_count = 0;
    }
}

} // ntk

namespace ntk
{

void SyncEventListener :: newEvent(EventBroadcaster* sender, EventDataPtr data)
{
    if (!m_enabled) return;
    m_lock.lock();
    Event new_event (sender, data);

    bool sender_in_queue = false;
    for (int i = 0; i < m_unprocessed_ordered_events.size(); ++i)
    {
        if (m_unprocessed_ordered_events[i].sender == sender)
        {
            // A event was received some time ago and left unprocessed.
            // Leave it at its place in the queue, but update the data
            // since there is no buffer it does not make sense to discard
            // the new data and process the old one.
            m_unprocessed_ordered_events[i] = new_event;
            sender_in_queue = true;
            break;
        }
    }

    if (!sender_in_queue)
    {
        m_unprocessed_ordered_events.push_back(new_event);
    }

    m_condition.wakeAll();
    m_lock.unlock();
}

void SyncEventListener :: setEnabled(bool enabled)
{
    m_enabled = enabled;
    m_condition.wakeAll();
}

EventListener::Event SyncEventListener :: waitForNewEvent(int timeout_msecs)
{
    if (!m_enabled) return 0;

    Event last_event;

    m_lock.lock();
    if (m_unprocessed_ordered_events.size() == 0)
    {
        m_condition.wait(&m_lock, timeout_msecs);
    }

    if (m_unprocessed_ordered_events.size() > 0)
    {
        last_event = m_unprocessed_ordered_events.front();
        m_unprocessed_ordered_events.pop_front();
    }

    m_lock.unlock();
    return last_event;
}

void AsyncEventListener :: newEvent(EventBroadcaster* sender, EventDataPtr data)
{
    SyncEventListener::newEvent(sender, data);

    {
        if (m_event_signaled)
            return;

        m_event_signaled = true;
    }

    if (!m_handler_running)
    {
        QApplication::postEvent(this, new EventBroadcasterUpdated(QEvent::User, sender, data), Qt::LowEventPriority);
    }
}

void AsyncEventListener :: customEvent(QEvent* generic_event)
{
    if (generic_event->type() != QEvent::User)
        return QObject::customEvent(generic_event);

    generic_event->accept();
    m_handler_running = true;
    while (m_event_signaled)
    {
        m_event_signaled = false;
        Event event = waitForNewEvent(1000);
        handleAsyncEvent(event);
        // FIXME: this is important on Windows to avoid the application
        // spending all its time handling these custom events.
        QApplication::processEvents();
    }
    m_handler_running = false;
}

void EventBroadcaster :: addEventListener(EventListener* updater)
{
    m_listeners.push_back(updater);
}

void EventBroadcaster :: removeAllEventListeners()
{
    m_listeners.clear();
}

void EventBroadcaster :: broadcastEvent(EventDataPtr data)
{
    foreach_idx(i, m_listeners)
            m_listeners[i]->newEvent(this, data);
}

}  // ntk

