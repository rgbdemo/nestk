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

#ifndef ROS_RGBD_GRABBER_H
#define ROS_RGBD_GRABBER_H

#include <ntk/camera/rgbd_grabber.h>

#include <ntk/core.h>
#include <ntk/utils/qt_utils.h>
#include <ntk/camera/calibration.h>
#include <ntk/thread/event.h>

#include <QThread>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>

namespace ntk
{

    /*!
     * Grab RGB-D images from ROS openni_camera.
     */
    class RosRGBDGrabber : public RGBDGrabber
    {
    public:
        RosRGBDGrabber(ros::NodeHandle nh) :
                m_nh(nh),
		m_image_transport(m_nh),
                m_depth_transmitted(true),
                m_rgb_transmitted(true)
        {}

        /*! Connect with ROS node. */
        bool initialize();

    protected:
        virtual void run();

    private:
        void depthCallBack(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);
        void rgbCallBack(const sensor_msgs::ImageConstPtr& msg);
        void fillCalibrationData(const sensor_msgs::CameraInfoConstPtr& info);

    private:
        ros::NodeHandle m_nh;
        image_transport::CameraSubscriber m_depth_sub;
        image_transport::Subscriber m_rgb_sub;
        image_transport::ImageTransport m_image_transport;
        bool m_depth_transmitted;
        bool m_rgb_transmitted;
        RGBDImage m_current_image;
    };

} // ntk

#endif // ROS_RGBD_GRABBER_H
