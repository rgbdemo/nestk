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

#ifndef NTK_THREAD_EVENT_H
# define NTK_THREAD_EVENT_H

#include <ntk/core.h>
#include <ntk/thread/utils.h>
#include <ntk/utils/debug.h>

#include <QWriteLocker>
#include <QWaitCondition>
#include <QMutex>
#include <QEvent>

#include <string>
#include <list>
#include <deque>

namespace ntk
{

class EventBroadcaster;

struct EventData;

ntk_ptr_typedefs(EventData)

//------------------------------------------------------------------------------

/*! Base class for objects that can be transmitted through events. */
struct CV_EXPORTS EventData
{
    virtual ~EventData();

public:
    virtual EventDataPtr clone () const;
};

#define TYPEDEF_THIS(Class) \
private:                    \
    typedef Class This;     \
public:

#define CLONABLE_EVENT_DATA                          \
public:                                              \
    virtual ::ntk::EventDataPtr clone () const       \
    {                                                \
        return ::ntk::EventDataPtr(new This(*this)); \
    }                                                \

//------------------------------------------------------------------------------

class EventListener
{
public:
    struct Event
    {
        Event(EventBroadcaster* sender = 0, EventDataPtr data = EventDataPtr())
            : sender(sender), data(data)
        {}

        bool isNull() const { return sender == 0; }
#if 0
        Event (const Event& copy)
        : sender(copy.sender)
        , data(copy.data->clone())
        {

        }

        Event& operator= (const Event& rhs)
        {
            Event that(rhs);
            this->swap(that);
            return *this;
        }

        void swap (Event& other)
        {
            std::swap(sender, other.sender);
            std::swap(data, other.data);
        }
#endif
        EventBroadcaster* sender;
        EventDataPtr data;
    };

public:
    EventListener();

public:
    virtual void newEvent(EventBroadcaster* sender = 0, EventDataPtr data = EventDataPtr()) = 0;

public:
    double frameRate() const { return m_framerate; }

protected:
    void reportNewEventProcessed();

private:
    uint64 m_last_frame_tick;
    double m_framerate;
    int m_frame_count;
};

//------------------------------------------------------------------------------

/*!
 * Listen to events and allows subclasses to waitForEvents.
 * There is no buffer, only one event will be kept from each source.
 * Implement a new BufferedSyncEventListener class if you want buffering.
 */
class SyncEventListener : public EventListener
{
public:
    SyncEventListener() :
        m_enabled(true)
    {}

    void setEnabled(bool enabled);
    bool enabled() const { return m_enabled; }

    virtual void newEvent(EventBroadcaster* sender = 0, EventDataPtr data = EventDataPtr());

    static const int event_timeout_msecs;
    Event waitForNewEvent(int timeout_msecs = event_timeout_msecs);

protected:
    int currentQueueSize() const;

private:
    bool m_enabled;
    mutable QMutex m_lock;
    QWaitCondition m_condition;
    std::deque<Event> m_unprocessed_ordered_events;
};

//------------------------------------------------------------------------------

class EventBroadcasterUpdated : public QObject, public QEvent
{
    Q_OBJECT

public:
    EventBroadcasterUpdated(QEvent::Type type, EventBroadcaster* broadcaster, EventDataPtr data)
        : QEvent(type), m_broadcaster(broadcaster), m_data(data)
    {
    }

    EventBroadcaster* broadcaster() { return m_broadcaster; }
    const EventBroadcaster* broadcaster() const { return m_broadcaster; }

    EventDataPtr data() { return m_data; }
    EventDataConstPtr data() const { return m_data; }

protected:
    EventBroadcaster* m_broadcaster;
    EventDataPtr m_data;
};

//------------------------------------------------------------------------------

class AsyncEventListener : public QObject, public SyncEventListener
{
    Q_OBJECT

public:
    AsyncEventListener() : m_event_signaled(false), m_handler_running(false)
    {}

    virtual void newEvent(EventBroadcaster* sender = 0, EventDataPtr data = EventDataPtr());
    virtual void handleAsyncEvent(Event event) = 0;
    virtual void customEvent(QEvent* event);

private:
    bool m_event_signaled; // FIXME: need for a lock here?
    bool m_handler_running;
};

class EventBroadcaster
{
public:
    virtual void addEventListener    (EventListener* listener);
    virtual void removeEventListener (EventListener* listener);
    virtual void removeAllEventListeners();
    virtual void broadcastEvent(EventDataPtr data = EventDataPtr());

private:
    typedef std::vector<EventListener*> Listeners;
    Listeners m_listeners;
};

//------------------------------------------------------------------------------

class EventProcessingBlockInOwnThread : public ntk::Thread, public SyncEventListener, public EventBroadcaster
{
    friend class EventProcessingBlockInOwnThreadDebugger;

    Q_OBJECT

public:
    typedef QString Name;

public:
     EventProcessingBlockInOwnThread (Name name = "unknown");
    ~EventProcessingBlockInOwnThread ();

signals:
    void dbg_started    (EventProcessingBlockInOwnThread*);
    void dbg_finished   (EventProcessingBlockInOwnThread*);
    void dbg_terminated (EventProcessingBlockInOwnThread*);

public slots:
    void on_started    ();
    void on_finished   ();
    void on_terminated ();

protected:
    const Name name;
};

//------------------------------------------------------------------------------

class EventProcessingBlockInOwnThreadDebugger : public QObject
{
    Q_OBJECT

public:
     EventProcessingBlockInOwnThreadDebugger ();
    ~EventProcessingBlockInOwnThreadDebugger ();

public slots:
    void constructed (EventProcessingBlockInOwnThread* that);
    void destroyed   (EventProcessingBlockInOwnThread* that);
    void started     (EventProcessingBlockInOwnThread* that);
    void finished    (EventProcessingBlockInOwnThread* that);
    void terminated  (EventProcessingBlockInOwnThread* that);
};

} // ntk

#endif // !NTK_THREAD_EVENT_H
