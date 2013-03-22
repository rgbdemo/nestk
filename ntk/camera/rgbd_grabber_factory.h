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

#ifndef GRABBER_FACTORY_H
#define GRABBER_FACTORY_H

#include <ntk/camera/rgbd_grabber.h>

namespace ntk
{

class OpenniDriver;
class Openni2Driver;
class Kin4WinDriver;

class RGBDGrabberFactory
{
public:
    enum enum_grabber_type { DEFAULT = 0, OPENNI = 1, FREENECT = 2, KIN4WIN = 3, PMD = 4, SOFTKINETIC = 5, SOFTKINETIC_IISU = 6, OPENNI2 = 7 };

    static enum_grabber_type getDefaultGrabberType();

    struct Params
    {
        Params();

        enum_grabber_type default_type;
        int camera_id;
        std::string directory;
        std::string image;
        std::string calibration_dir; // where calibration files can be found: path/calibration-SERIAL.yml
        std::string calibration_file;
        bool synchronous;
        bool track_users;
        bool high_resolution;
        bool hardware_registration;
        bool disable_openni2;
    };

    struct GrabberData
    {
        GrabberData() : type(DEFAULT), grabber(0), processor(0) {}
        enum_grabber_type type;
        RGBDGrabber* grabber;
        RGBDProcessor* processor;
        std::string message;
    };

private:
     RGBDGrabberFactory();

public:
     static RGBDGrabberFactory& instance ();

public:
     ~RGBDGrabberFactory();

public:
    std::vector<GrabberData> createGrabbers(const Params& params);

protected:
    bool createFileGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createOpenniGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createOpenni2Grabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createFreenectGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createKin4winGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createPmdGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createSoftKineticGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);
    bool createSoftKineticIisuGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers);

    RGBDProcessor* createProcessor(const enum_grabber_type& grabber_type);
    RGBDCalibrationPtr tryLoadCalibration(const Params &params, const std::string &camera_serial);

protected:
    OpenniDriver* ni_driver;
    Openni2Driver* ni2_driver;
    Kin4WinDriver* kin4win_driver;
};

}

#endif // GRABBER_FACTORY_H
