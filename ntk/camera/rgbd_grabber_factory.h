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
    };

    struct GrabberData
    {
        GrabberData() : type(DEFAULT), grabber(0), processor(0) {}
        enum_grabber_type type;
        RGBDGrabber* grabber;
        RGBDProcessor* processor;
        std::string message;
    };

public:
     RGBDGrabberFactory();
    ~RGBDGrabberFactory();

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
