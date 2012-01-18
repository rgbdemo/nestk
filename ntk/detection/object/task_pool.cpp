//
// TaskPool.cpp
//
// Author: Nicolas Burrus <nicolas.burrus@ensta.fr>, (C) 2007
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//

#include "task_pool.h"
#include <ntk/ntk.h>

namespace ntk
{

  Task::Task(const std::string& name)
      : taskName(name)
  {
  }

  Task::~Task()
  {
  }

  const std::string Task::getTaskName() const
  {
    return taskName;
  }

  void Task::setTaskName(const std::string& newName)
  {
    taskName = newName;
  }

} // end of ntk

namespace ntk
{

  class TaskPoolThread  : public ntk::Thread
  {
    TaskPool& pool;
    bool volatile busy;

    TaskPoolThread(const TaskPoolThread&);
    const TaskPoolThread& operator= (const TaskPoolThread&);

    public:
      TaskPoolThread(TaskPool& pool_)
          : Thread(),
          pool(pool_),
          busy(false)
      {
      }

      ~TaskPoolThread()
      {
      }

      void run()
      {
        while (! threadShouldExit())
        {
          if (! pool.runNextTask())
            waitForNotification(100);
        }
      }
  };

} // end of ntk

namespace ntk
{

  //==============================================================================
  TaskPool::TaskPool(const int numThreads, const QThread::Priority priority)
      : numThreads(numThreads),
      m_paused(false),
      m_has_done_work(false)
  {
    ntk_assert(numThreads > 0, "Must have at least one thread");
    m_threads.resize(numThreads);

    for (int i = numThreads; --i >= 0;)
    {
      m_threads[i] = new TaskPoolThread(*this);
      m_threads[i]->start();
    }

    setThreadPriorities(priority);
  }

  TaskPool::~TaskPool()
  { 
    foreach_idx(i, m_threads)
    {
      m_threads[i]->setThreadShouldExit();
    }

    foreach_idx(i, m_threads)
    {
      m_threads[i]->wait(4000);
      if (!m_threads[i]->isFinished())
      {
        m_threads[i]->terminate();
        m_threads[i]->wait();
      }
      delete m_threads[i];
    }
  }

  void TaskPool::addTask(Task* const task)
  {
    lock.lock();
    m_waiting_tasks.push(task);
    lock.unlock();

    for (int i = numThreads; --i >= 0;)
      m_threads[i]->notify();
  }

  int TaskPool::getNumTasks() const throw()
  {
    QMutexLocker slock(&lock);
    return m_running_tasks.size() + m_waiting_tasks.size();
  }

  int TaskPool::getNumRunningTasks() const throw()
  {
    QMutexLocker slock(&lock);
    return m_running_tasks.size();
  }

  void TaskPool :: waitForTasksToFinish() const
  {
    while (getNumTasks() > 0)
    {
      ntk::sleep(10);
    }
  }

  void TaskPool :: waitForRunningTasksToFinish() const
  {
    while (getNumRunningTasks() > 0)
    {
      ntk::sleep(10);
    }
  }
  
  void TaskPool::setThreadPriorities(const QThread::Priority newPriority)
  {
    for (int i = numThreads; --i >= 0;)
      m_threads[i]->setPriority(newPriority);
  }

  bool TaskPool::runNextTask()
  {
    lock.lock();

    if (m_paused)
    {
      lock.unlock();
      return false;
    }

    if (m_waiting_tasks.size() == 0)
    {
      if (m_has_done_work && m_running_tasks.size() == 0)
      {
        m_has_done_work = 0;
        lock.unlock();
        broadcastEvent();
      }
      else
      {
        lock.unlock();
      }
      return false;
    }

    Task* task = m_waiting_tasks.front(); m_waiting_tasks.pop();
    m_running_tasks.insert(task);
    lock.unlock();

    try
    {
      Task::TaskStatus result = task->runTask();

      const QMutexLocker sl(&lock);

      // Only delete the first occurrence, there might be several ones.
      m_running_tasks.erase(m_running_tasks.find(task));
      m_has_done_work = true;

      if (result == Task::taskNeedsRunningAgain)
      {
        addTask(task);
      }
      else
      {
        if (result == Task::taskHasFinishedAndShouldBeDeleted)
          delete task;
      }
    }
    catch (...)
    {
      lock.lock();
      m_running_tasks.erase(m_running_tasks.find(task));
      lock.unlock();
    }
    return true;
  }

  void TaskPool :: pause()
  {
    lock.lock();
    m_paused = true;
    lock.unlock();

    for (int i = numThreads; --i >= 0;)
      m_threads[i]->notify();

    waitForRunningTasksToFinish();
  }

  void TaskPool :: resume()
  {
    lock.lock();
    m_paused = false;
    lock.unlock();

    for (int i = numThreads; --i >= 0;)
      m_threads[i]->notify();
  }

} // end of avs
