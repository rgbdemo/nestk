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
 * Author: Nicolas Tisserand <nicolas.tisserand@manctl.com>
 */

#ifndef NTK_CAMERA_KIN4WIN_GRABBER_H
# define NTK_CAMERA_KIN4WIN_GRABBER_H

#include <ntk/camera/rgbd_grabber.h>
#include <ntk/gesture/skeleton.h>

namespace ntk
{

class Nui;

class Kin4WinDriver
{
public:
    struct DeviceInfo
    {
        std::string creation_info;
        std::string camera_type;
        std::string serial;
        std::string vendor;
        unsigned short vendor_id;
        unsigned short product_id;
        unsigned char bus;
        unsigned char address;
    };

public:
    static bool hasDll ();

public:
    Kin4WinDriver();
    ~Kin4WinDriver();

public:
    int numDevices() const { return devices_info.size(); }
    const DeviceInfo& deviceInfo(int index) const { return devices_info[index]; }

private:
    std::vector<DeviceInfo> devices_info;
};

class Kin4WinGrabber : public ntk::RGBDGrabber
{
    friend class Nui;

public:
    Kin4WinGrabber(Kin4WinDriver& driver, int camera_id = 0);
    Kin4WinGrabber(Kin4WinDriver& driver, const std::string& camera_serial);
    virtual ~Kin4WinGrabber();

public:
    virtual std::string grabberType () const { return "kin4win"; }

    virtual bool connectToDevice();
    virtual bool disconnectFromDevice();
    virtual void setNearMode(bool enable);

    /*! Set whether color images should be in high resolution 1280x1024. */
    void setHighRgbResolution(bool hr) { m_high_resolution = hr; }
    bool getHighRgbResolution() const { return  m_high_resolution; }
    bool getAccelerometerXYZ(cv::Point3f& xyz) const;

protected:
    virtual void run();

private:
    void estimateCalibration();
    void updateCalibrationMinMaxDepth ();

private:
    Nui* nui; // FIXME: Handle multiple device setups.
    Kin4WinDriver& m_driver;
    int m_camera_id;
    std::string m_camera_serial;
    RGBDImage m_current_image;
    static QMutex m_ni_mutex;
    bool m_near_mode;
    bool m_near_mode_changed;
    bool m_high_resolution;
    bool m_align_depth_to_color;
    bool m_is_xbox_kinect;
};

}

#endif // !NTK_CAMERA_KIN4WIN_GRABBER_H
