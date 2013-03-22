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


#include "multiple_grabber.h"

ntk::MultipleGrabber::~MultipleGrabber()
{
    stop();

    foreach_idx(i, m_image_listeners)
    {
        delete m_image_listeners[i];
    }
}

void ntk::MultipleGrabber::setAlternativeDisconnectMode(bool enable)
{
    if (m_disconnect_alternatively == enable)
        return;

    stop();
    disconnectFromDevice();
    m_disconnect_alternatively = enable;
    connectToDevice();
    start();
}

void ntk::MultipleGrabber::addGrabber(RGBDGrabber* grabber)
{
    m_grabbers.push_back(grabber);
    ImageListener* listener = new ImageListener(this);
    grabber->addEventListener(listener);
    m_image_listeners.push_back(listener);
    m_temp_grabbed_images.resize(m_grabbers.size());
    m_grabbed_images.resize(m_grabbers.size());
}

std::string ntk::MultipleGrabber::grabberType() const
{
    if (m_grabbers.size() < 1)
        return "unknown";

    if (m_grabbers.size() == 1)
        return m_grabbers[0]->grabberType ();

    return "multiple";
}

bool ntk::MultipleGrabber::connectToDevice()
{
    bool ok = true;
    foreach_idx(i, m_grabbers)
    {
        ok &= m_grabbers[i]->connectToDevice();
    }
    return ok;
}

bool ntk::MultipleGrabber::disconnectFromDevice()
{
    bool ok = true;
    foreach_idx(i, m_grabbers)
    {
        ok &= m_grabbers[i]->disconnectFromDevice();
    }
    return ok;
}

void ntk::MultipleGrabber::onImageUpdated(EventBroadcaster *sender)
{
    RGBDGrabber* grabber = dynamic_cast<RGBDGrabber*>(sender);
    int grabber_index = 0;
    while (m_grabbers[grabber_index] != grabber) ++grabber_index;

    m_all_grabbers_updated_mutex.lock();
    grabber->copyImageTo(m_temp_grabbed_images[grabber_index]);
    m_updated_grabbers.insert(grabber);

    if (m_updated_grabbers.size() == m_grabbers.size())
    {
        m_all_grabbers_updated_condition.wakeAll();
    }
    else if (m_disconnect_alternatively)
    {
        m_grabbers[grabber_index]->stop();
        m_grabbers[grabber_index]->disconnectFromDevice();

        for (int i = 0; i < m_grabbers.size(); ++i)
        {
            if (m_updated_grabbers.find(m_grabbers[i]) == m_updated_grabbers.end())
            {
                m_grabbers[i]->connectToDevice();
                m_grabbers[i]->start();
                break;
            }
        }
    }

    m_all_grabbers_updated_mutex.unlock();   
}

void ntk::MultipleGrabber::copyImageTo(RGBDImage& image)
{
    QReadLocker locker(&m_lock);
    m_grabbed_images[0].copyTo(image);
}

void ntk::MultipleGrabber::copyImagesTo(std::vector<RGBDImage>& images)
{
    QReadLocker locker(&m_lock);
    images.resize(m_grabbed_images.size());
    foreach_idx(i, m_grabbed_images)
    {
        m_grabbed_images[i].copyTo(images[i]);
    }
}

void ntk::MultipleGrabber::run()
{
    setThreadShouldExit(false);

    if (m_grabbers.size() < 1)
    {
        ntk_throw_exception("No grabbers!");
    }

    if (!m_disconnect_alternatively)
    {
        foreach_idx(i, m_grabbers)
        {
            m_grabbers[i]->start();
        }
    }
    else // m_disconnect_alternatively
    {
        for (int i = 0; i < m_grabbers.size(); ++i)
        {
            m_grabbers[i]->disconnectFromDevice();
        }
        m_grabbers[0]->connectToDevice();
        m_grabbers[0]->start();
    }

    while (!threadShouldExit())
    {
        if (isSynchronous())
        {
            waitForNewEvent();
            foreach_idx(i, m_grabbers)
            {
                m_grabbers[i]->newEvent();
            }
        }

        m_all_grabbers_updated_mutex.lock();
        m_all_grabbers_updated_condition.wait(&m_all_grabbers_updated_mutex);
        m_updated_grabbers.clear();

        {
            QWriteLocker locker(&m_lock);
            m_grabbed_images.swap(m_temp_grabbed_images);
        }

        m_all_grabbers_updated_mutex.unlock();
        advertiseNewFrame();
    }
}

void ntk::MultipleGrabber::ImageListener::newEvent(EventBroadcaster *sender)
{
    grabber->onImageUpdated(sender);
}

void ntk::MultipleGrabber::setSynchronous(bool sync)
{
    SyncEventListener::setEnabled(sync);
    foreach_idx(i, m_grabbers)
    {
        m_grabbers[i]->setSynchronous(sync);
    }
}

void ntk::MultipleGrabber::stop()
{
    RGBDGrabber::stop();
    foreach_idx(i, m_grabbers)
    {
        m_grabbers[i]->stop();
    }
}
