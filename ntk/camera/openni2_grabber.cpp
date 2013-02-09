#include "openni2_grabber.h"
#include <OpenNI2/OpenNI.h>
#include <brief/impl.h>
#include <set>
#include <iterator>
#include <cassert>
#include <cstdio>

using namespace openni;

//------------------------------------------------------------------------------

namespace ntk {

Openni2Driver::SensorInfo::SensorInfo (const DeviceInfo& info)
    : uri(info.getUri())
    , vendor(info.getVendor())
    , name(info.getName())
    , vendorId(info.getUsbVendorId())
    , productId(info.getUsbProductId())
    , key(uri + vendor + name + vendorId + productId)
{

}

bool
Openni2Driver::hasDll ()
{
#ifdef _MSC_VER
    // Trigger OpenNI2 SDK DLL loading by calling one of its functions.
    __try
    {
        OniStatus status = oniInitialize(ONI_API_VERSION);
        // FIXME: Call something without side effects above and remove below.
        if (ONI_STATUS_OK == status)
            oniShutdown();

        return true;
    }
    __except(EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
#else
    return true;
#endif
}

class Openni2Driver::Impl
        : public OpenNI::DeviceConnectedListener
        , public OpenNI::DeviceDisconnectedListener
        , public OpenNI::DeviceStateChangedListener
{
public:
     Impl (Openni2Driver* that)
         : that(that)
     {
         Status rc = OpenNI::initialize();

         if (rc != STATUS_OK)
         {
             ntk_error("Initialize failed: %s\n", OpenNI::getExtendedError());
             return;
         }

         Array<DeviceInfo> devices;
         OpenNI::enumerateDevices(&devices);

         for (int i = 0; i < devices.getSize(); ++i)
         {
             ntk_info("Device \"%s\" connected.\n", devices[i].getUri());
             infos.insert(SensorInfo(devices[i]));
         }

         OpenNI::addDeviceConnectedListener(this);
         OpenNI::addDeviceDisconnectedListener(this);
         OpenNI::addDeviceStateChangedListener(this);
     }

    ~Impl ()
    {
        OpenNI::removeDeviceConnectedListener(this);
        OpenNI::removeDeviceDisconnectedListener(this);
        OpenNI::removeDeviceStateChangedListener(this);

        OpenNI::shutdown();
    }

public:
    virtual void onDeviceStateChanged (const DeviceInfo* pInfo, DeviceState state)
    {
        ntk_info("Device \"%s\" state changed to %d.\n", pInfo->getUri(), state);
    }

    virtual void onDeviceConnected (const DeviceInfo* info)
    {
        ntk_info("Device \"%s\" connected.\n", info->getUri());
        infos.insert(SensorInfo(*info));
    }

    virtual void onDeviceDisconnected (const DeviceInfo* info)
    {
        ntk_info("Device \"%s\" disconnected.\n", info->getUri());
        infos.erase(infos.find(SensorInfo(*info)));
    }

    SensorInfos
    getSensorInfos () const
    {
        return SensorInfos(infos.begin(), infos.end());
    }

private:
    Openni2Driver* that;
    std::set<SensorInfo> infos;
};

FWD_IMPL_0_CONST(Openni2Driver::SensorInfos, Openni2Driver, getSensorInfos)

Openni2Driver::Openni2Driver ()
    : impl(new Impl(this))
{

}

Openni2Driver::~Openni2Driver ()
{
    delete impl;
}

}

//------------------------------------------------------------------------------

namespace {

void analyzeFrame(const VideoFrameRef& frame)
{
    DepthPixel* pDepth;
    RGB888Pixel* pColor;

    int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;

    switch (frame.getVideoMode().getPixelFormat())
    {
    case PIXEL_FORMAT_DEPTH_1_MM:
    case PIXEL_FORMAT_DEPTH_100_UM:
        pDepth = (DepthPixel*)frame.getData();
        ntk_info("[%08llu] %8d\n", (long long)frame.getTimestamp(),
            pDepth[middleIndex]);
        break;
    case PIXEL_FORMAT_RGB888:
        pColor = (RGB888Pixel*)frame.getData();
        ntk_info("[%08llu] 0x%02x%02x%02x\n", (long long)frame.getTimestamp(),
            pColor[middleIndex].r&0xff,
            pColor[middleIndex].g&0xff,
            pColor[middleIndex].b&0xff);
        break;
    default:
        ntk_info("Unknown format\n");
    }
}

class FrameCallback : public VideoStream::NewFrameListener
{
public:
    void onNewFrame(VideoStream& stream)
    {
        stream.readFrame(&m_frame);

        analyzeFrame(m_frame);
    }
private:
    VideoFrameRef m_frame;
};

}

//------------------------------------------------------------------------------

namespace ntk {

Openni2Grabber::Openni2Grabber (Openni2Driver& driver, int camera_id)
    : m_driver(driver)
{
    // FIXME: Implement.
}

Openni2Grabber::Openni2Grabber (Openni2Driver& driver, const std::string& camera_serial)
    : m_driver(driver)
{
    // FIXME: Implement.
}

bool
Openni2Grabber::connectToDevice ()
{
    // FIXME: Implement.
    return false;
}

bool
Openni2Grabber::disconnectFromDevice ()
{
    // FIXME: Implement.
    return false;
}

void
Openni2Grabber::setSubsamplingFactor (int factor)
{
    // FIXME: Implement.
}

void
Openni2Grabber::setIRMode (bool ir)
{
    // FIXME: Implement.
}

void
Openni2Grabber::run ()
{
    while (true)
        ; // FIXME: Implement.

    ntk::Openni2Driver driver;

    Device device;
    Status rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        ntk_error("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return;
    }

    VideoStream depth;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
            ntk_error("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    rc = depth.start();
    if (rc != STATUS_OK)
    {
        ntk_error("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    }


    FrameCallback frameCallback;

    depth.addNewFrameListener(&frameCallback);

    // Wait while we're getting frames through the printer
//    while (!wasKeyboardHit())
//    {
//        Sleep(100);
//    }

    depth.removeNewFrameListener(&frameCallback);

    depth.stop();
    depth.destroy();
    device.close();
}

}
