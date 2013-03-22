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

#ifndef   	NTK_UTILS_TIME_H_
# define   	NTK_UTILS_TIME_H_

# include <ntk/core.h>
# include <ntk/utils/debug.h>
# include <QMutex>
# include <QWaitCondition>
# include <iostream>
# include <string>
# include <sstream>

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
    static float getMicrosecondCounter() { return 1000000.0*cv::getTickCount()/cv::getTickFrequency(); }
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
        if (m_debug_level <= ntk_debug_level)
            full_text << "[" << marker << " = " << delta << "ms]";
        return delta;
    }

    uint64 elapsedMsecsNoPrint() const
    {
        uint64 delta = ntk::Time::getMillisecondCounter()-m_start;
        return delta;
    }

    void stop(const std::string& marker = "")
    {
      uint64 delta = ntk::Time::getMillisecondCounter() - m_start;
      ntk_dbg(m_debug_level) << "[TIME] elapsed in " << m_name << marker << ": " << delta << " ms " << full_text.str();
    }

  private:
    std::string m_name;
    mutable std::ostringstream full_text;
    uint64 m_start;
    int m_debug_level;
  };

  class FrameRate
  {
  public:
      FrameRate()
          : m_last_reference_tick(),
            m_frame_counter(0),
            m_frame_rate(-1)
      {}

  public:
      enum { delta_frames = 10 };
      void tick()
      {
          m_last_tick = cv::getTickCount();

          if (m_frame_counter == 0)
          {
              m_last_reference_tick = m_last_tick;
              ++m_frame_counter;
              return;
          }

          if (m_frame_counter < delta_frames)
          {
              double current_tick = m_last_tick;
              m_frame_rate = m_frame_counter / ((current_tick - m_last_reference_tick)/cv::getTickFrequency());
              ++m_frame_counter;
              return;
          }

          if (m_frame_counter % delta_frames == 0)
          {
              double current_tick = m_last_tick;

              m_frame_rate = delta_frames / ((current_tick - m_last_reference_tick)/cv::getTickFrequency());
              m_last_reference_tick = current_tick;
          }

          ++m_frame_counter;
      }

      float currentFrameRate()
      {
          return m_frame_rate;
      }

      bool goingSlowerThan (float target_fps) const
      {
          if (m_frame_counter == 0)
              return true;

          double current_tick = cv::getTickCount();
          double frame_rate = 1.0 / ((current_tick - m_last_tick)/cv::getTickFrequency());
          return frame_rate < target_fps;
      }

  private:
      double m_last_reference_tick;
      double m_last_tick;
      int m_frame_counter;
      float m_frame_rate;
  };

} // end of ntk

#endif 	    /* !NTK_UTILS_TIME_H_ */
