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

#ifndef   	NTK_THREAD_TASKPOOL_H_
# define   	NTK_THREAD_TASKPOOL_H_

# include <ntk/core.h>
# include <ntk/thread/event.h>

# include <queue>
# include <set>

namespace ntk
{

  class TaskPoolThread; class TaskPool;

  class Task
  {
  public:
    Task (const std::string& name);
    virtual ~Task();

    const std::string getTaskName() const;
    void setTaskName (const std::string& newName);

    enum TaskStatus
      {
        taskHasFinished = 0,
        taskHasFinishedAndShouldBeDeleted,
        taskNeedsRunningAgain
      };

    virtual TaskStatus runTask() = 0;

  private:
    std::string taskName;
  };

  class TaskPool : public ntk::EventBroadcaster
  {
  public:
    TaskPool (const int numberOfThreads,
              const QThread::Priority priority = QThread::NormalPriority);
    ~TaskPool();

  public:
    void addTask (Task* const task);

    int getNumTasks() const throw();
    int getNumRunningTasks() const throw();

    void setThreadPriorities (const QThread::Priority newPriority);

    void waitForTasksToFinish() const;
    void waitForRunningTasksToFinish() const;
    
  public:
    void pause();

    void resume();

  private:
    const int numThreads;
    bool m_paused;
    bool m_has_done_work;
    std::vector<ntk::Thread*> m_threads;
    std::queue<Task*> m_waiting_tasks;
    std::multiset<Task*> m_running_tasks;

    mutable RecursiveQMutex lock;

    friend class TaskPoolThread;
    bool runNextTask();

    TaskPool (const TaskPool&);
    const TaskPool& operator= (const TaskPool&);
  };

} // end of avs

#endif 	    /* !NTK_THREAD_TASKPOOL_H_ */
