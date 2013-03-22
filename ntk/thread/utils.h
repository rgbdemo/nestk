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
