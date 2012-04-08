#include "kin4win_grabber.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/gesture/body_event.h>
#include <ntk/geometry/pose_3d.h>

#include <QDateTime>
#include <string>
#include <fstream>

using namespace cv;
using namespace ntk;

QMutex ntk::Kin4WinGrabber::m_ni_mutex;

namespace
{

}

namespace ntk
{

Kin4WinGrabber :: Kin4WinGrabber(Kin4WinDriver& driver, int camera_id) :
    m_driver(driver),
    m_camera_id(camera_id)
{

}

Kin4WinGrabber :: Kin4WinGrabber(Kin4WinDriver& driver, const std::string& camera_serial) :
    m_driver(driver),
    m_camera_id(-1),
    m_camera_serial(camera_serial)
{
    for (size_t i = 0; i < driver.numDevices(); ++i)
    {
        if (driver.deviceInfo(i).serial == camera_serial)
        {
            m_camera_id = i;
            break;
        }
    }

    if (m_camera_id < 0)
    {
        ntk_throw_exception("Could not find any device with serial " + camera_serial);
    }
}

bool Kin4WinGrabber :: connectToDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_dbg(1) << format("[Kinect %x] connecting", this);

    return false;
}

bool Kin4WinGrabber :: disconnectFromDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_dbg(1) << format("[Kinect %x] disconnecting", this);

    return false;
}

void Kin4WinGrabber :: estimateCalibration()
{

}

void Kin4WinGrabber :: run()
{

}

} // ntk

ntk::Kin4WinDriver::Kin4WinDriver()
{
    ntk_dbg(1) << "Initializing Kin4Win driver";

    if (ntk::ntk_debug_level >= 1)
    {

    }

    if (ntk::ntk_debug_level >= 2)
    {

    }

    // ntk_throw_exception(format("enumerating devices failed. Reason: %s", xnGetStatusString(status)));

    // ntk_dbg(1) << format("Found device: vendor %s name %s", description.strVendor, description.strName);

    // ntk_dbg(1) << cv::format("[Device %d] %s, %s, serial=%s",
}

ntk::Kin4WinDriver :: ~Kin4WinDriver()
{

}
