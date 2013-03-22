
#include "rgbd_grabber.h"
#include <ntk/utils/time.h>
#include <QMutexLocker>

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

    if (m_target_framerate > 0)
    {
        float target_delta = m_frame_count * 1000.f/m_target_framerate;
        if (delta_tick < target_delta)
        {
            ntk::sleep(target_delta - delta_tick);
        }
    }

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
    setThreadShouldExit();
    newEvent();
    wait();
}

}

int ntk::RGBDGrabber::getCurrentTimestamp()
{
    if (m_initial_timestamp == 0)
    {
        m_initial_timestamp = ntk::Time::getMillisecondCounter();
        return 0.f;
    }
    else
    {
        return (ntk::Time::getMillisecondCounter() - m_initial_timestamp);
    }
}

void ntk::RGBDGrabber::setThreadShouldExit (bool should_exit)
{
    QMutexLocker _(&mutex);
    m_should_exit = should_exit;
}

bool ntk::RGBDGrabber::threadShouldExit () const
{
    QMutexLocker _(const_cast<QMutex*>(&mutex));
    return m_should_exit;
}
