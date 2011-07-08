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

#ifndef NTK_CAMERA_NITE_RGBD_GRABBER_H
#define NTK_CAMERA_NITE_RGBD_GRABBER_H

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/gesture/skeleton.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnVSessionManager.h>
#include <XnVPushDetector.h>

namespace ntk
{

class BodyEventDetector;

class OpenniGrabber : public ntk::RGBDGrabber
{
public:
    OpenniGrabber(int camera_id = 0) :
        m_camera_id(camera_id),
        m_need_pose_to_calibrate(false),
        m_max_num_users(15),
        m_body_event_detector(0),
        m_high_resolution(false),
        m_mirrored(false),
        m_custom_bayer_decoding(true),
        m_xml_config_file(DEFAULT_XML_CONFIG_FILE),
        m_track_users(true)
    {
    }

    /*! set a new xml config file for the grabber
   * call it before initialize() */
    void set_xml_config_file(const std::string & xml_filename);

    /*! Call it before starting the thread. */
    virtual bool connectToDevice();

    /*! Disconnect from Kinect. */
    virtual bool disconnectFromDevice();

    /*! Set the body event detector. */
    void setBodyEventDetector(BodyEventDetector* detector)
    { m_body_event_detector = detector; }

    /*! Set the maximal number of tracked users. Default is one. */
    void setMaxUsers(int num) { m_max_num_users = num; }

    /*! Set whether color images should be in high resolution 1280x1024. */
    void setHighRgbResolution(bool hr) { m_high_resolution = hr; }

    /*! Set whether images should be vertically mirrored. */
    void setMirrored(bool m) { m_mirrored = m; }

    /*! Set whether User and Body trackers are enabled. */
    void setTrackUsers(bool enable) { m_track_users = enable; }

public:
    // Nite accessors.
    xn::DepthGenerator& niDepthGenerator() { return m_ni_depth_generator; }
    xn::UserGenerator& niUserGenerator() { return m_ni_user_generator; }
    const xn::DepthGenerator& niDepthGenerator() const { return m_ni_depth_generator; }
    const xn::UserGenerator& niUserGenerator() const { return m_ni_user_generator; }

protected:
    /*! Thread loop. */
    virtual void run();

public:
    /*! Callback: New user was detected */
    void newUserCallback(XnUserID nId);

    /*! Callback: An existing user was lost */
    void lostUserCallback(XnUserID nId);

    /*! Callback: Detected a pose */
    void userPoseDetectedCallback(XnUserID nId);

    /*! Callback: Calibration started */
    void calibrationStartedCallback(XnUserID nId);

    /*! Callback: Finished calibration */
    void calibrationFinishedCallback(XnUserID nId, bool success);

private:
    void check_error(const XnStatus& status, const char* what) const;
    void estimateCalibration();

private:
    int m_camera_id;
    RGBDImage m_current_image;
    xn::Context m_ni_context;
    xn::DepthGenerator m_ni_depth_generator;
    xn::ImageGenerator m_ni_rgb_generator;
    xn::UserGenerator m_ni_user_generator;
    xn::HandsGenerator m_ni_hands_generator;
    xn::GestureGenerator m_ni_gesture_generator;
    XnLicense license;
    bool m_need_pose_to_calibrate;
    XnChar m_calibration_pose[20];
    int m_max_num_users;
    BodyEventDetector* m_body_event_detector;
    bool m_high_resolution;
    bool m_mirrored;
    bool m_custom_bayer_decoding;

    /*! the default xml config file */
    static const std::string DEFAULT_XML_CONFIG_FILE;

    /*! the xml config file to load in initialize() */
    std::string m_xml_config_file;

    bool m_track_users;

    static QMutex m_ni_mutex;
};

// Backward compatibility.
typedef OpenniGrabber NiteRGBDGrabber;

} // ntk

#endif // NTK_CAMERA_NITE_RGBD_GRABBER_H
