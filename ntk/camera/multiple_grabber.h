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

#ifndef NTK_CAMERA_MULTIPLE_GRABBER_H
#define NTK_CAMERA_MULTIPLE_GRABBER_H

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/thread/event.h>

#include <set>

namespace ntk
{

class MultipleGrabber : public RGBDGrabber
{
public:
    struct ImageListener : public SyncEventListener
    {
        ImageListener(MultipleGrabber* grabber) : grabber (grabber) {}
        virtual void newEvent(EventBroadcaster* sender);
        MultipleGrabber* grabber;
    };

public:
    MultipleGrabber() : m_disconnect_alternatively (false) {}
    virtual ~MultipleGrabber();

public:
    /*!
     * Set whether only one device will be connected at one time.
     * This is useful when using multiple kinects to avoid inteferences.
     */
    void setAlternativeDisconnectMode(bool enable);
    void addGrabber(RGBDGrabber* grabber);

    virtual std::string grabberType () const;

public:
    virtual bool connectToDevice();
    virtual bool disconnectFromDevice();
    virtual void copyImagesTo(std::vector<RGBDImage>& images);
    virtual void copyImageTo(RGBDImage& image);
    virtual void setSynchronous(bool sync);

public:
    virtual void onImageUpdated(EventBroadcaster* sender);

protected:
    /*! Thread loop. */
    virtual void run();
    virtual void stop();

private:
    QMutex m_all_grabbers_updated_mutex;
    QWaitCondition m_all_grabbers_updated_condition;
    std::vector<RGBDGrabber*> m_grabbers;
    std::vector<ImageListener*> m_image_listeners;
    std::vector<RGBDImage> m_temp_grabbed_images;
    std::vector<RGBDImage> m_grabbed_images;
    std::set<RGBDGrabber*> m_updated_grabbers;
    bool m_disconnect_alternatively;
};

} // ntk

#endif // NTK_CAMERA_MULTIPLE_GRABBER_H
