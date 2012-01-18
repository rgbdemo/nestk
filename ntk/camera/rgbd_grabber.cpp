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

#include "rgbd_grabber.h"

#include <ntk/utils/time.h>

namespace ntk
{

void RGBDGrabber :: copyImageTo(RGBDImage& image)
{
    QReadLocker locker(&m_lock);
    m_rgbd_image.copyTo(image);
}

void RGBDGrabber :: copyImagesTo(std::vector<RGBDImage>& images)
{
    QReadLocker locker(&m_lock);
    images.resize(1);
    m_rgbd_image.copyTo(images[0]);
}

void RGBDGrabber :: advertiseNewFrame()
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

    m_condition.wakeAll();
    broadcastEvent();
}

void RGBDGrabber :: stop()
{
    setShouldExit();
    newEvent();
    wait();
}

}

float ntk::RGBDGrabber::getCurrentTimestamp()
{
    if (m_initial_timestamp == 0)
    {
        m_initial_timestamp = ntk::Time::getMillisecondCounter();
        return 0.f;
    }
    else
    {
        return (ntk::Time::getMillisecondCounter() - m_initial_timestamp) / 1000.f;
    }
}
