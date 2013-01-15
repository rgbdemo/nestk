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

RGBDProcessor* RGBDGrabberFactory::createProcessor(const enum_grabber_type& grabber_type)
{
    switch (grabber_type)
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

bool RGBDGrabberFactory :: createOpenniGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifndef NESTK_USE_OPENNI
    return false;
#else
    if (!OpenniDriver::hasDll())
        return false;

    // Config dir is supposed to be next to the binaries.
    QDir prev = QDir::current();
    QDir::setCurrent(QApplication::applicationDirPath());
    OpenniDriver* ni_driver = new OpenniDriver();

    ntk_dbg_print(ni_driver->numDevices(), 1);

    // Create grabbers.
    for (int i = 0; i < ni_driver->numDevices(); ++i)
    {
        OpenniGrabber* k_grabber = new OpenniGrabber(*ni_driver, i);
        k_grabber->setTrackUsers(false);
        if (params.high_resolution)
            k_grabber->setHighRgbResolution(true);

        k_grabber->setCustomBayerDecoding(false);
        if (params.hardware_registration)
            k_grabber->setUseHardwareRegistration(false);

        GrabberData new_data;
        new_data.grabber = k_grabber;
        new_data.type = OPENNI;
        new_data.processor = createProcessor(OPENNI);
        grabbers.push_back(new_data);
    }

    QDir::setCurrent(prev.absolutePath());

    return ni_driver->numDevices() > 0;
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
    return false;
#endif
}


bool RGBDGrabberFactory :: createKin4winGrabbers(const ntk::RGBDGrabberFactory::Params &params, std::vector<GrabberData>& grabbers)
{
#ifdef NESTK_USE_KIN4WIN

    if (!Kin4WinDriver::hasDll())
        return false;

    std::vector<std::string> calibration_files;

    Kin4WinDriver* kin_driver = new Kin4WinDriver;

    if (kin_driver->numDevices() < 1)
        return false;

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
            ntk_dbg(0) << "Warning, directory " << camera_path << " has no images, skipping.";
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

RGBDCalibration* RGBDGrabberFactory::tryLoadCalibration(const ntk::RGBDGrabberFactory::Params &params,
                                                        const std::string& camera_serial)
{
    ntk::RGBDCalibration* calib_data = 0;

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
        ntk_dbg(0) << "Warning: could not load calibration file " << filename;
        if (calib_data)
            delete_and_zero(calib_data);
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
        createPmdGrabbers(params, grabbers);
        createSoftKineticGrabbers(params, grabbers);
    }

    foreach_idx(i, grabbers)
    {
        GrabberData& data = grabbers[i];
        if (params.synchronous)
            data.grabber->setSynchronous(true);
        RGBDCalibration* calibration = tryLoadCalibration(params, data.grabber->cameraSerial());
        if (calibration)
            data.grabber->setCalibrationData(*calibration);
        data.processor = createProcessor(data.type);
    }

    return grabbers;
}

} // ntk
