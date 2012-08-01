#include "rgbd_grabber_factory.h"

#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/camera/rgbd_processor.h>

#ifdef NESTK_USE_OPENNI
# include <ntk/camera/openni_grabber.h>
#endif

#ifdef NESTK_USE_FREENECT
# include <ntk/camera/freenect_grabber.h>
#endif

#ifdef NESTK_USE_KIN4WIN
# include <ntk/camera/kin4win_grabber.h>
#endif

#ifdef NESTK_USE_PMDSDK
# include <ntk/camera/pmd_grabber.h>
#endif

#include <QApplication>

namespace ntk
{

RGBDGrabberFactory::Params::Params()
    : type(RGBDGrabberFactory::getDefaultGrabberType()),
      camera_id(0),
      synchronous(false),
      track_users(false),
      high_resolution(false)
{

}

RGBDGrabberFactory::grabber_type
RGBDGrabberFactory::getDefaultGrabberType()
{
#ifdef NESTK_USE_OPENNI
    return OPENNI;
#endif

#ifdef NESTK_USE_FREENECT
    return FREENECT;
#endif

#ifdef NESTK_USE_KIN4WIN
    return KIN4WIN;
#endif

#ifdef NESTK_USE_PMDSDK
    return PMD;
#endif

    return OPENNI;
}

RGBDGrabberFactory::RGBDGrabberFactory()
    : ni_driver(0),
      kin4win_driver(0)
{
}

RGBDProcessor* RGBDGrabberFactory::createProcessor(const Params& params)
{
    switch (params.type)
    {
    case OPENNI:
    case KIN4WIN:
        return new OpenniRGBDProcessor;

    case FREENECT:
        return new FreenectRGBDProcessor;

#ifdef NESTK_USE_PMDSDK
    case PMD:
        return new PmdRgbProcessor;
#endif

    default:
        return 0;
    }
}

RGBDGrabberFactory::GrabberData RGBDGrabberFactory::createFileGrabber(const Params& params)
{
    GrabberData data;
    std::string path = params.directory.empty() ? params.image : params.directory;
    FileGrabber* file_grabber = new FileGrabber(path, !params.directory.empty());
    data.grabber = file_grabber;
    data.processor = createProcessor(params);
    return data;
}

RGBDGrabberFactory::GrabberData RGBDGrabberFactory::createOpenNIGrabber(const Params& params)
{
    GrabberData data;
#ifdef NESTK_USE_OPENNI
    // Config dir is supposed to be next to the binaries.
    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    if (!ni_driver) ni_driver = new OpenniDriver();

    if (ni_driver->numDevices() < 1)
    {
        data.message = "No device connected.";
        return data;
    }

    OpenniGrabber* k_grabber = new OpenniGrabber(*ni_driver, params.camera_id);
    k_grabber->setTrackUsers(params.track_users);
    k_grabber->setHighRgbResolution(params.high_resolution);
    bool ok = k_grabber->connectToDevice();
    QDir::setCurrent(prev.absolutePath());

    if (!ok)
    {
        data.message = "Could not connect to OpenNI device.";
        delete k_grabber;
        return data;
    }

    data.grabber = k_grabber;
    data.processor = createProcessor(params);
#else
    data.message = "OpenNI support not built.";
#endif
    return data;
}

RGBDGrabberFactory::GrabberData RGBDGrabberFactory::createFreenectGrabber(const Params& params)
{
    GrabberData data;
#ifdef NESTK_USE_FREENECT
    FreenectGrabber* k_grabber = new FreenectGrabber();
    bool ok = k_grabber->connectToDevice();
    if (!ok)
    {
        data.message = "Cannot connect to any freenect device.";
        delete k_grabber;
        return data;
    }

    data.grabber = k_grabber;
    data.processor = createProcessor(params);
#else
    data.message = "Freenect support not built.";
#endif
    return data;
}

RGBDGrabberFactory::GrabberData RGBDGrabberFactory::createKin4winGrabber(const Params& params)
{
    GrabberData data;
#ifdef NESTK_USE_KIN4WIN
    if (!kin4win_driver)
        kin4win_driver = new Kin4WinDriver;

    Kin4WinGrabber* k_grabber = new Kin4WinGrabber(*kin4win_driver);
    bool ok = k_grabber->connectToDevice();
    if (!ok)
    {
        data.message = "Cannot connect to any freenect device.";
        delete k_grabber;
        return data;
    }

    data.grabber = k_grabber;
    data.processor = createProcessor(params);
#else
    data.message = "Kinect for Windows support not build.";
#endif
    return data;
}

RGBDGrabberFactory::GrabberData RGBDGrabberFactory::createPmdGrabber(const Params& params)
{
    GrabberData data;
#ifdef NESTK_USE_PMDSDK
    PmdGrabber* k_grabber = new PmdGrabber();
    bool ok = k_grabber->connectToDevice();
    if (!ok)
    {
        data.message = "Cannot connect to any PMD device.";
        delete k_grabber;
        return data;
    }

    data.grabber = k_grabber;
    data.processor = createProcessor(params);
#else
    data.message = "PMD support not built.";
#endif
    return data;
}

RGBDCalibration* RGBDGrabberFactory::tryLoadCalibration(const Params& params)
{
    ntk::RGBDCalibration* calib_data = 0;
    if (!params.calibration_file.empty())
    {
        calib_data = new RGBDCalibration();
        calib_data->loadFromFile(params.calibration_file.c_str());
    }
    return calib_data;
}

std::vector<RGBDGrabberFactory::GrabberData>
RGBDGrabberFactory::createGrabbers(const ntk::RGBDGrabberFactory::Params &params)
{
    std::vector<GrabberData> grabbers;

    GrabberData data;

    if (!params.image.empty() || !params.directory.empty())
    {
        data = createFileGrabber(params);
    }
    else
    {
        switch (params.type)
        {
        case OPENNI:
        {
            data = createOpenNIGrabber(params);
            break;
        }

        case FREENECT:
        {
            data = createFreenectGrabber(params);
            break;
        }

        case KIN4WIN:
        {
            data = createKin4winGrabber(params);
            break;
        }

        case PMD:
        {
            data = createPmdGrabber(params);
            break;
        }
        };
    }

    if (data.grabber)
    {
        if (params.synchronous)
            data.grabber->setSynchronous(true);
        RGBDCalibration* calibration = tryLoadCalibration(params);
        if (calibration)
            data.grabber->setCalibrationData(*calibration);
        grabbers.push_back(data);
    }
    else
    {
        ntk_dbg(0) << "[ERROR] Could not create grabber: " << data.message;
    }

    return grabbers;
}

} // ntk
