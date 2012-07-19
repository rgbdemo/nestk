#ifndef GRABBER_FACTORY_H
#define GRABBER_FACTORY_H

#include <ntk/camera/rgbd_grabber.h>

namespace ntk
{

class OpenniDriver;
class Kin4WinDriver;

class RGBDGrabberFactory
{
public:
    enum grabber_type { DEFAULT = 0, OPENNI = 1, FREENECT = 2, KIN4WIN = 3, PMD = 4 };

    static grabber_type getDefaultGrabberType();

    struct Params
    {
        Params();

        grabber_type type;
        int camera_id;
        std::string directory;
        std::string image;
        std::string calibration_file;
        bool synchronous;
        bool track_users;
        bool high_resolution;
    };

    struct GrabberData
    {
        GrabberData() : type(DEFAULT), grabber(0), processor(0) {}
        grabber_type type;
        RGBDGrabber* grabber;
        RGBDProcessor* processor;
        std::string message;
    };

public:
    RGBDGrabberFactory();

    std::vector<GrabberData> createGrabbers(const Params& params);

protected:
    GrabberData createFileGrabber(const Params& params);
    GrabberData createOpenNIGrabber(const Params& params);
    GrabberData createFreenectGrabber(const Params& params);
    GrabberData createKin4winGrabber(const Params& params);
    GrabberData createPmdGrabber(const Params& params);
    RGBDProcessor* createProcessor(const Params& params);
    RGBDCalibration* tryLoadCalibration(const Params& params);

protected:
    OpenniDriver* ni_driver;
    Kin4WinDriver* kin4win_driver;
};

}

#endif // GRABBER_FACTORY_H
