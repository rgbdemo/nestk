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

#include "ros_rgbd_grabber.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

    void RosRGBDGrabber :: fillCalibrationData(const sensor_msgs::CameraInfoConstPtr& info)
    {
        m_calib_data = new RGBDCalibration();
        m_calib_data->setRawRgbSize(cv::Size(info->width, info->height));
        m_calib_data->setRgbSize(cv::Size(info->width, info->height));
        m_calib_data->raw_depth_size = m_calib_data->raw_rgb_size;
        m_calib_data->depth_size = m_calib_data->raw_depth_size;

        // What happens in high res mode ?
        float rgb_fx = info->K[0];
        float rgb_fy = info->K[4];
        float rgb_cx = info->K[2];
        float rgb_cy = info->K[5];
        float depth_fx = info->K[0];
        float depth_fy = info->K[4];
        float depth_cx = info->K[2];
        float depth_cy = info->K[5];

        m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
        setIdentity(m_calib_data->rgb_intrinsics);
        m_calib_data->rgb_intrinsics(0,0) = rgb_fx;
        m_calib_data->rgb_intrinsics(1,1) = rgb_fy;
        m_calib_data->rgb_intrinsics(0,2) = rgb_cx;
        m_calib_data->rgb_intrinsics(1,2) = rgb_cy;

        m_calib_data->rgb_distortion = Mat1d(1,5);
        m_calib_data->rgb_distortion = 0.;
        m_calib_data->zero_rgb_distortion = true;

        // After getAlternativeViewpoint, both camera have the same parameters.

        m_calib_data->depth_intrinsics = cv::Mat1d(3,3);
        setIdentity(m_calib_data->depth_intrinsics);
        m_calib_data->depth_intrinsics(0,0) = depth_fx;
        m_calib_data->depth_intrinsics(1,1) = depth_fy;
        m_calib_data->depth_intrinsics(0,2) = depth_cx;
        m_calib_data->depth_intrinsics(1,2) = depth_cy;

        m_calib_data->depth_distortion = Mat1d(1,5);
        m_calib_data->depth_distortion = 0.;
        m_calib_data->zero_depth_distortion = true;

        m_calib_data->R = Mat1d(3,3);
        setIdentity(m_calib_data->R);

        m_calib_data->T = Mat1d(3,1);
        m_calib_data->T = 0.;

        m_calib_data->depth_pose = new Pose3D();
        m_calib_data->depth_pose->setCameraParametersFromOpencv(m_calib_data->depth_intrinsics);

        m_calib_data->rgb_pose = new Pose3D();
        m_calib_data->rgb_pose->toRightCamera(m_calib_data->rgb_intrinsics,
                                              m_calib_data->R,
                                              m_calib_data->T);

        m_calib_data->camera_type = "kinect-ni";
        m_current_image.setCalibration(m_calib_data);
    }

    void RosRGBDGrabber :: depthCallBack(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info)
    {
        if (m_calib_data == 0)
        {
            fillCalibrationData(info);
        }

        m_current_image.rawDepthRef().create(msg->height, msg->width);
        std::copy(msg->data.begin(), msg->data.end(), m_current_image.rawDepthRef().data);
        for_all_rc(m_current_image.rawDepth())
        {
            if (isnan(m_current_image.rawDepth()(r,c)))
                m_current_image.rawDepthRef()(r,c) = 0;
        }

        m_depth_transmitted = false;
    }

    void RosRGBDGrabber :: rgbCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        m_current_image.rawRgbRef().create(msg->height, msg->width);
        std::copy(msg->data.begin(), msg->data.end(), m_current_image.rawRgbRef().data);
        cvtColor(m_current_image.rawRgb(), m_current_image.rawRgbRef(), CV_RGB2BGR);
        m_rgb_transmitted = false;
    }

    bool RosRGBDGrabber :: initialize()
    {
        m_nh.setCallbackQueue(new ros::CallbackQueue());

        std::string topic = m_nh.resolveName("depth_in");
        image_transport::TransportHints transport("raw");
        m_depth_sub = m_image_transport.subscribeCamera(topic, 1, &RosRGBDGrabber::depthCallBack, this, transport);

        topic = m_nh.resolveName("rgb_in");
        m_rgb_sub = m_image_transport.subscribe(topic, 1, &RosRGBDGrabber::rgbCallBack, this, transport);

	return false;
    }

    void RosRGBDGrabber :: run()
    {
        m_should_exit = false;
        m_current_image.setCalibration(m_calib_data);
        m_rgbd_image.setCalibration(m_calib_data);

#if 0
        m_rgbd_image.rawRgbRef() = Mat3b(FREENECT_FRAME_H, FREENECT_FRAME_W);
        m_rgbd_image.rawDepthRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);
        m_rgbd_image.rawIntensityRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);

        m_current_image.rawRgbRef() = Mat3b(FREENECT_FRAME_H, FREENECT_FRAME_W);
        m_current_image.rawDepthRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);
        m_current_image.rawIntensityRef() = Mat1f(FREENECT_FRAME_H, FREENECT_FRAME_W);
#endif

        int64 last_grab_time = 0;

        ros::Rate r(30); // 30 hz
        while (!m_should_exit)
        {
            waitForNewEvent();
            while (m_depth_transmitted || m_rgb_transmitted)
            {
                ROS_DEBUG("Spinning");
                ros::spinOnce();
                r.sleep();
            }
            ROS_DEBUG("Send new frame");

            {
                int64 grab_time = ntk::Time::getMillisecondCounter();
                ntk_dbg_print(grab_time - last_grab_time, 2);
                last_grab_time = grab_time;
                QWriteLocker locker(&m_lock);
                m_current_image.swap(m_rgbd_image);
                m_rgb_transmitted = true;
                m_depth_transmitted = true;
            }
            advertiseNewFrame();
        }
    }

} // ntk
