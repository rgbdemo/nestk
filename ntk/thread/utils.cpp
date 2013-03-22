
#include "utils.h"

#include <ntk/utils/debug.h>

#include <iostream>

namespace ntk
{

void Thread::stopThread()
{
    // FIXME: setThreadShouldExit has to be called repeatedly because
    // the thread can be running but the run function might not have
    // been called yet. And most run() function reset threadShouldExit.
    while (isRunning())
    {
        setThreadShouldExit(true);
        wait(100);
    }
}

void Thread :: waitForNotification(int timeout_msecs)
{
    m_mutex.lock();
    m_wait_condition.wait(&m_mutex, timeout_msecs);
    m_mutex.unlock();
}

} // ntk

