#include "rgbd_grabber_factory.h"

#include <ntk/camera/opencv_grabber.h>
#include <ntk/camera/file_grabber.h>
#include <ntk/camera/rgbd_frame_recorder.h>
#include <ntk/camera/rgbd_processor.h>

#ifdef NESTK_USE_OPENNI
# include <ntk/camera/openni_grabber.h>
#endif

#ifdef NESTK_USE_OPENNI2
# include <ntk/camera/openni2_grabber.h>
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

#ifdef NESTK_USE_SOFTKINETIC
# include <ntk/camera/softkinetic_grabber.h>
#endif

#ifdef NESTK_USE_SOFTKINETIC_IISU
# include <ntk/camera/softkinetic_iisu_grabber.h>
#endif

#include <QApplication>

namespace ntk
{

RGBDGrabberFactory::Params::Params()
    : default_type(RGBDGrabberFactory::getDefaultGrabberType()),
      camera_id(0),
      synchronous(false),
      track_users(false),
      high_resolution(false),
      hardware_registration(true)
{

}

RGBDGrabberFactory::enum_grabber_type
RGBDGrabberFactory::getDefaultGrabberType()
{
#ifdef NESTK_USE_OPENNI
    return OPENNI;
#endif

#ifdef NESTK_USE_OPENNI2
    return OPENNI2;
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
    : ni_driver(0)
    , ni2_driver(0)
    , kin4win_driver(0)
{
}

RGBDGrabberFactory::~RGBDGrabberFactory()
{
#ifdef NESTK_USE_KIN4WIN
    delete kin4win_driver;
#endif

#ifdef NESTK_USE_OPENNI2
    delete ni2_driver;
#endif

#ifdef NESTK_USE_OPENNI
    delete ni_driver;
#endif
}


RGBDProcessor* RGBDGrabberFactory::createProcessor(const enum_grabber_type& grabber_type)
{
    switch (grabber_type)
    {
    case OPENNI:
        return new OpenniRGBDProcessor;

    case OPENNI2:
        // FIXME: Is this valid?
        return new OpenniRGBDProcessor;

    case KIN4WIN:
        return new Kin4winRGBDProcessor;

    case SOFTKINETIC:
        return new SoftKineticRGBDProcessor;

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

bool RGBDGrabberFactory :: createOpenniGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_OPENNI
    if (!OpenniDriver::hasDll())
    {
        ntk_warn("No OpenNI dll found.\n");
        return false;
    }

    // Config dir is supposed to be next to the binaries.
    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    ni_driver = new OpenniDriver();

    ntk_info("Number of Openni devices: %d\n", ni_driver->numDevices());
    ntk_dbg_print(ni_driver->numDevices(), 1);

    // Create grabbers.
    for (int i = 0; i < ni_driver->numDevices(); ++i)
    {
        OpenniGrabber* k_grabber = new OpenniGrabber(*ni_driver, i);
        k_grabber->setTrackUsers(false);
        if (params.high_resolution)
            k_grabber->setHighRgbResolution(true);

        k_grabber->setCustomBayerDecoding(false);
        k_grabber->setUseHardwareRegistration(params.hardware_registration);

        GrabberData new_data;
        new_data.grabber = k_grabber;
        new_data.type = OPENNI;
        new_data.processor = createProcessor(OPENNI);
        grabbers.push_back(new_data);
    }

    QDir::setCurrent(prev.absolutePath());

    return ni_driver->numDevices() > 0;
#else
    return false;
#endif
}

bool RGBDGrabberFactory :: createOpenni2Grabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_OPENNI2
    if (!Openni2Driver::hasDll())
    {
        ntk_warn("No OpenNI2 dll found.\n");
        return false;
    }

    ni2_driver = new Openni2Driver();

    Openni2Driver::SensorInfos sensorInfos = ni2_driver->getSensorInfos();

    ntk_info("Number of Openni devices: %d\n", sensorInfos.size());
    ntk_dbg_print(sensorInfos.size(), 1);

    // Create grabbers.
    for (unsigned n = 0; n < sensorInfos.size(); ++n)
    {
        Openni2Grabber* grabber = new Openni2Grabber(*ni2_driver, n);

        if (params.high_resolution)
            grabber->setHighRgbResolution(true);

        grabber->setCustomBayerDecoding(false);
        grabber->setUseHardwareRegistration(params.hardware_registration);

        GrabberData data;
        data.grabber = grabber;
        data.type = OPENNI2;
        data.processor = createProcessor(OPENNI2);

        grabbers.push_back(data);
    }

    return sensorInfos.size() > 0;
#else
    return false;
#endif
}

bool RGBDGrabberFactory :: createPmdGrabbers(const ntk::RGBDGrabberFactory::Params &paramss, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_PMDSDK
    std::vector<std::string> calibration_files;

    // Config dir is supposed to be next to the binaries.
    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    PmdGrabber* k_grabber = new PmdGrabber();
    if (!k_grabber->connectToDevice())
    {
        delete k_grabber;
        return false;
    }

    GrabberData new_data;
    new_data.grabber = k_grabber;
    new_data.type = PMD;
    new_data.processor = createProcessor(PMD);
    grabbers.push_back(new_data);

    return true;
#else
    return false;
#endif
}

bool RGBDGrabberFactory :: createSoftKineticGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_SOFTKINETIC
    if (!SoftKineticGrabber::hasDll())
    {
        ntk_warn("No softkinetic DepthSense dll found.\n");
        return false;
    }

    ntk_info("Trying softkinetic backend\n");

    std::vector<std::string> calibration_files;

    // Config dir is supposed to be next to the binaries.
    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    SoftKineticGrabber* k_grabber = new SoftKineticGrabber();
    if (!k_grabber->connectToDevice())
    {
        delete k_grabber;
        return false;
    }

    GrabberData new_data;
    new_data.grabber = k_grabber;
    new_data.type = SOFTKINETIC;
    grabbers.push_back(new_data);
    return true;
#else
    ntk_info("No support for softkinetic, skipping.\n");
    return false;
#endif
}

bool RGBDGrabberFactory :: createSoftKineticIisuGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_SOFTKINETIC_IISU
    ntk_dbg(1) << "Trying softkinetic iisu backend";

    std::vector<std::string> calibration_files;

    // Config dir is supposed to be next to the binaries.
    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());

    SoftKineticIisuGrabber* k_grabber = new SoftKineticIisuGrabber();
    if (!k_grabber->connectToDevice())
    {
        delete k_grabber;
        return false;
    }

    GrabberData new_data;
    new_data.grabber = k_grabber;
    new_data.type = SOFTKINETIC_IISU;
    grabbers.push_back(new_data);
    return true;
#else
    ntk_dbg(1) << "No support for softkinetic iisu, skipping.";
    return false;
#endif
}

bool RGBDGrabberFactory :: createKin4winGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_KIN4WIN

    if (!Kin4WinDriver::hasDll())
    {
        ntk_warn("No Kinect for Windows SDK dll found.\n");
        return false;
    }

    std::vector<std::string> calibration_files;

    kin4win_driver = new Kin4WinDriver;

    if (kin_driver->numDevices() < 1)
    {
        ntk_info("No Kinect for Windows devices found.\n");
        return false;
    }

    ntk_info("Number of Kinect for Windows devices found: %d.\n", kin_driver->numDevices());

    // Create grabbers.
    for (int i = 0; i < kin_driver->numDevices(); ++i)
    {
        Kin4WinGrabber* k_grabber = new Kin4WinGrabber(*kin_driver, i);

        if (params.high_resolution)
            k_grabber->setHighRgbResolution(true);

        GrabberData new_data;
        new_data.grabber = k_grabber;
        new_data.type = KIN4WIN;
        grabbers.push_back(new_data);
    }

    return true;
#else
    ntk_info ("No support for Kinect for Windows SDK.\n");
    return false;
#endif
}

bool RGBDGrabberFactory :: createFileGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
    std::string path = params.directory;

    std::vector<std::string> image_directories;

    QDir root_path (path.c_str());
    QStringList devices = root_path.entryList(QStringList("*"), QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    foreach (QString name, devices)
    {
        QString camera_path = root_path.absoluteFilePath(name);
        if (QDir(camera_path).entryList(QStringList("view*"), QDir::Dirs, QDir::Name).size() == 0)
        {
            ntk_warn("Warning, directory %s has no images, skipping.\n", camera_path.toAscii().constData());
            continue;
        }
        image_directories.push_back(camera_path.toStdString());
    }

    if (image_directories.size() < 1)
        return false;

    grabbers.resize(image_directories.size());

    for (size_t i = 0; i < image_directories.size(); ++i)
    {
        FileGrabber* file_grabber = new FileGrabber(image_directories[i], true);
        GrabberData new_data;
        new_data.grabber = file_grabber;
        new_data.type = params.default_type;
        grabbers[i] = new_data;
    }

    return true;
}

RGBDCalibrationPtr RGBDGrabberFactory::tryLoadCalibration(const ntk::RGBDGrabberFactory::Params &params,
                                                          const std::string& camera_serial)
{
    ntk::RGBDCalibrationPtr calib_data;

    std::string filename;
    try
    {
        if (!params.calibration_file.empty())
        {
            filename = params.calibration_file.c_str();
        }
        else if (!params.calibration_dir.empty())
        {
            filename = cv::format("%s/calibration-%s.yml",
                                  params.calibration_dir.c_str(),
                                  camera_serial.c_str()).c_str();
        }

        if (!filename.empty())
        {
            calib_data = new RGBDCalibration();
            calib_data->loadFromFile(filename.c_str());
        }
    }
    catch (const std::exception& e)
    {
        ntk_warn("Warning: could not load calibration file %s\n", filename.c_str());
    }

    return calib_data;
}

std::vector<RGBDGrabberFactory::GrabberData>
RGBDGrabberFactory::createGrabbers(const ntk::RGBDGrabberFactory::Params& orig_params)
{
    Params params = orig_params;

    bool from_disk = !params.directory.empty();

    std::vector<RGBDGrabberFactory::GrabberData> grabbers;

    if (from_disk)
    {
        // By default look for calibration files in the same directory as images.
        if (params.calibration_dir.empty() && params.calibration_file.empty())
            params.calibration_dir = params.directory;
        createFileGrabbers(params, grabbers);
    }
    else
    {
        createKin4winGrabbers(params, grabbers);
        createOpenniGrabbers(params, grabbers);
        createOpenni2Grabbers(params, grabbers);
        createPmdGrabbers(params, grabbers);
        createSoftKineticGrabbers(params, grabbers);
        createSoftKineticIisuGrabbers(params, grabbers);
    }

    foreach_idx(i, grabbers)
    {
        GrabberData& data = grabbers[i];
        if (params.synchronous)
            data.grabber->setSynchronous(true);
        RGBDCalibrationPtr calibration = tryLoadCalibration(params, data.grabber->cameraSerial());
        if (calibration)
            data.grabber->setCalibrationData(calibration);
        data.processor = createProcessor(data.type);
    }

    return grabbers;
}

} // ntk
