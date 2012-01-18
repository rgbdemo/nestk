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

#ifndef   	NTK_UTILS_TIME_H_
# define   	NTK_UTILS_TIME_H_

# include <ntk/core.h>
# include <ntk/utils/debug.h>
# include <QMutex>
# include <QWaitCondition>
# include <iostream>
# include <string>

namespace ntk
{
  
  inline void sleep(int msecs)
  {
    // FIXME: really hacky!
    QMutex mutex; mutex.lock();
    QWaitCondition cond;
    cond.wait(&mutex, msecs);
    mutex.unlock();
  }

  class Time
  {
  public:
    static uint64 getMillisecondCounter() { return 1000.0*cv::getTickCount()/cv::getTickFrequency(); }
  };

  class TimeCount
  {
  public:
    TimeCount(const std::string& name, int debug_level = 1)
      : m_name(name),
        m_start(ntk::Time::getMillisecondCounter()),
        m_debug_level(debug_level)
    {
    }

    uint64 elapsedMsecs(const std::string& marker = "") const
    {
        uint64 delta = ntk::Time::getMillisecondCounter()-m_start;
        ntk_dbg(m_debug_level) << "[TIME (step)] elapsed in " << m_name << marker << delta << " msecs";
        return delta;
    }

    void stop(const std::string& marker = "")
    {
      uint64 delta = ntk::Time::getMillisecondCounter() - m_start;
      ntk_dbg(m_debug_level) << "[TIME] elapsed in " << m_name << marker << ": " << delta << " msecs";
    }

  private:
    std::string m_name;
    uint64 m_start;
    int m_debug_level;
  };

  class FrameRate
  {
  public:
      FrameRate()
          : m_last_tick(),
            m_frame_counter(0),
            m_frame_rate(-1)
      {}

  public:
      void tick()
      {
          const int delta_frames = 10;

          if (m_frame_counter == 0)
          {
              m_last_tick = cv::getTickCount();
              ++m_frame_counter;
              return;
          }

          if (m_frame_counter < delta_frames)
          {
              double current_tick = cv::getTickCount();
              m_frame_rate = m_frame_counter / ((current_tick - m_last_tick)/cv::getTickFrequency());
              ++m_frame_counter;
              return;
          }

          if (m_frame_counter % delta_frames == 0)
          {
              double current_tick = cv::getTickCount();

              m_frame_rate = delta_frames / ((current_tick - m_last_tick)/cv::getTickFrequency());
              m_last_tick = current_tick;
          }

          ++m_frame_counter;
      }

      float currentFrameRate()
      {
          return m_frame_rate;
      }

  private:
      double m_last_tick;
      int m_frame_counter;
      float m_frame_rate;
  };

} // end of ntk

#endif 	    /* !NTK_UTILS_TIME_H_ */
