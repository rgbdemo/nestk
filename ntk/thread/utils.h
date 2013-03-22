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

#ifndef NTK_THREAD_UTILS_H
#define NTK_THREAD_UTILS_H

# include <ntk/core.h>
# include <QMutex>
# include <QReadWriteLock>
# include <QThread>
# include <QWaitCondition>

class RecursiveQMutex : public QMutex
{
public:
  RecursiveQMutex() : QMutex(RecursiveQMutex::Recursive)
  {}
};

class RecursiveQReadWriteLock : public QReadWriteLock
{
public:
  RecursiveQReadWriteLock() : QReadWriteLock(QReadWriteLock::Recursive)
  {}
};

namespace ntk
{

class Thread : public QThread
{
public:
  Thread() : QThread(), m_thread_should_exit(false)
  {}

public:
  void stopThread();

  void setThreadShouldExit(bool thread_should_exit = true) { m_thread_should_exit = thread_should_exit; }

  bool threadShouldExit() const { return m_thread_should_exit; }

  void waitForNotification(int timeout_msecs = 500);

  void notify() { m_wait_condition.wakeAll(); }

private:
  bool m_thread_should_exit;
  QWaitCondition m_wait_condition;
  mutable QMutex m_mutex;
};

template <class T>
class LockedResource
{
public:
    LockedResource(T* ptr = 0) : ptr(ptr) {}

public:
    T* acquire() { mutex.lock(); return ptr; }
    void release() { mutex.unlock(); }

private:
    T* ptr;
    QMutex mutex;
};

} // ntk

#endif // NTK_THREAD_UTILS_H
