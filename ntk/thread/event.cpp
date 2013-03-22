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


#include "event.h"
#include <ntk/utils/time.h>

#include <QApplication>
#include <QMutex>
#include <algorithm>

namespace ntk
{

EventData::~EventData()
{
}

EventDataPtr EventData::clone() const
{
    // not implemented, but do not make it pure virtual to avoid
    // breaking compatibility with existing code.   
    abort();
    return EventDataPtr();
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

//------------------------------------------------------------------------------

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

const int
SyncEventListener::event_timeout_msecs = 100;

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

int SyncEventListener::currentQueueSize() const
{
    m_lock.lock();
    int value = m_unprocessed_ordered_events.size();
    m_lock.unlock();
    return value;
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
    const int queue_size = currentQueueSize();
    for (int i = 0; i < queue_size; ++i)
    {
        Event event = waitForNewEvent();
        handleAsyncEvent(event);
        // FIXME: this is important on Windows to avoid the application
        // spending all its time handling these custom events.
        // FIXME: this seems to slow down QT for nothing.
        // QApplication::processEvents();
    }
    m_event_signaled = false;
    m_handler_running = false;
}

void EventBroadcaster :: addEventListener(EventListener* listener)
{
    if (m_listeners.end() != std::find(m_listeners.begin(), m_listeners.end(), listener))
        return;

    m_listeners.push_back(listener);
}

void EventBroadcaster :: removeEventListener(EventListener* listener)
{
    Listeners::iterator i = std::find(m_listeners.begin(), m_listeners.end(), listener);

    if (m_listeners.end() == i)
        return;

    m_listeners.erase(i);
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

//------------------------------------------------------------------------------

namespace ntk {

namespace {

EventProcessingBlockInOwnThreadDebugger* debuggerInstance = 0;
int debuggerCount = 0;
QMutex debuggerMutex;

#define SERIALIZED const QMutexLocker _(&debuggerMutex);

EventProcessingBlockInOwnThreadDebugger& debugger ()
{
    SERIALIZED

    if (0 == debuggerInstance)
        debuggerInstance = new EventProcessingBlockInOwnThreadDebugger;

    return *debuggerInstance;
}

}

EventProcessingBlockInOwnThreadDebugger::EventProcessingBlockInOwnThreadDebugger ()
{

}

EventProcessingBlockInOwnThreadDebugger::~EventProcessingBlockInOwnThreadDebugger ()
{

}

void EventProcessingBlockInOwnThreadDebugger::constructed (EventProcessingBlockInOwnThread* that)
{
    SERIALIZED

    // QTextStream qs(stdout);
    qDebug() << "EventProcessingBlockInOwnThread: constructed: " << that->name << endl;

    ++debuggerCount;
}

void EventProcessingBlockInOwnThreadDebugger::destroyed (EventProcessingBlockInOwnThread* that)
{
    SERIALIZED

    // QTextStream qs(stdout);
    qDebug() << "EventProcessingBlockInOwnThread: destroyed: " << that->name << endl;

    if (0 < --debuggerCount)
        return;

    delete this;
    debuggerInstance = 0;
}

void EventProcessingBlockInOwnThreadDebugger::started (EventProcessingBlockInOwnThread* that)
{
    SERIALIZED

    // QTextStream qs(stdout);
    qDebug() << "EventProcessingBlockInOwnThread: started: " << that->name << endl;
}

void EventProcessingBlockInOwnThreadDebugger::finished (EventProcessingBlockInOwnThread* that)
{
    SERIALIZED

    // QTextStream qs(stdout);
    qDebug() << "EventProcessingBlockInOwnThread: finished: " << that->name << endl;
}

void EventProcessingBlockInOwnThreadDebugger::terminated (EventProcessingBlockInOwnThread* that)
{
    SERIALIZED

    // QTextStream qs(stdout);
    qDebug() << "EventProcessingBlockInOwnThread: terminated: " << that->name << endl;
}

#undef SERIALIZED

EventProcessingBlockInOwnThread::EventProcessingBlockInOwnThread (Name name)
: name(name)
{
    debugger().constructed(this);

    QObject::connect(this, SIGNAL(dbg_started   (EventProcessingBlockInOwnThread*)), &debugger(), SLOT(started   (EventProcessingBlockInOwnThread*)));
    QObject::connect(this, SIGNAL(dbg_finished  (EventProcessingBlockInOwnThread*)), &debugger(), SLOT(finished  (EventProcessingBlockInOwnThread*)));
    QObject::connect(this, SIGNAL(dbg_terminated(EventProcessingBlockInOwnThread*)), &debugger(), SLOT(terminated(EventProcessingBlockInOwnThread*)));

    QObject::connect(this, SIGNAL(started   ()), this, SLOT(on_started   ()));
    QObject::connect(this, SIGNAL(finished  ()), this, SLOT(on_finished  ()));
    QObject::connect(this, SIGNAL(terminated()), this, SLOT(on_terminated()));
}

EventProcessingBlockInOwnThread::~EventProcessingBlockInOwnThread ()
{
    QObject::disconnect(this, SIGNAL(terminated()), this, SLOT(on_terminated()));
    QObject::disconnect(this, SIGNAL(finished  ()), this, SLOT(on_finished  ()));
    QObject::disconnect(this, SIGNAL(started   ()), this, SLOT(on_started   ()));

    QObject::disconnect(this, SIGNAL(dbg_terminated(EventProcessingBlockInOwnThread*)), &debugger(), SLOT(terminated(EventProcessingBlockInOwnThread*)));
    QObject::disconnect(this, SIGNAL(dbg_finished  (EventProcessingBlockInOwnThread*)), &debugger(), SLOT(finished  (EventProcessingBlockInOwnThread*)));
    QObject::disconnect(this, SIGNAL(dbg_started   (EventProcessingBlockInOwnThread*)), &debugger(), SLOT(started   (EventProcessingBlockInOwnThread*)));

    debugger().destroyed(this);
}

void
EventProcessingBlockInOwnThread::on_started ()
{
    emit dbg_started(this);
}

void
EventProcessingBlockInOwnThread::on_finished ()
{
    emit dbg_finished(this);
}

void
EventProcessingBlockInOwnThread::on_terminated ()
{
    emit dbg_terminated(this);
}

}
