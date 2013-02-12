#include "openni2_grabber.h"
#include <OpenNI2/OpenNI.h>
#include <brief/impl.h>
#include <set>
#include <iterator>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <QMutex>
#include <QMutexLocker>
#include <QString>

using namespace openni;

//------------------------------------------------------------------------------

namespace ntk {

Openni2Driver::SensorInfo::SensorInfo (const DeviceInfo& info)
    : uri(info.getUri())
    , vendor(info.getVendor())
    , name(info.getName())
    , vendorId(info.getUsbVendorId())
    , productId(info.getUsbProductId())
    , key(uri + vendor + name + QString(vendorId) + QString(productId))
{

}

bool
Openni2Driver::hasDll ()
{
#ifdef _MSC_VER
    // Trigger OpenNI2 SDK DLL loading by calling one of its functions.
    __try
    {
        const int version = OpenNi::getVersion();

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
         , ready(false)
     {
         QMutexLocker _(&mutex);

         openni::Status rc = OpenNI::initialize();

         if (rc != STATUS_OK)
         {
             ntk_error("OpenNI2: Initialize failed: %s\n", OpenNI::getExtendedError());
             return;
         }

         Array<DeviceInfo> devices;
         OpenNI::enumerateDevices(&devices);

         for (int i = 0; i < devices.getSize(); ++i)
         {
             ntk_info("OpenNI2: Device \"%s\" present.\n", devices[i].getUri());
             infos.insert(SensorInfo(devices[i]));
         }

         OpenNI::addDeviceConnectedListener(this);
         OpenNI::addDeviceDisconnectedListener(this);
         OpenNI::addDeviceStateChangedListener(this);

         ready = true;
     }

    ~Impl ()
    {
        QMutexLocker _(&mutex);

        ready = false;

        infos.clear();

        OpenNI::removeDeviceConnectedListener(this);
        OpenNI::removeDeviceDisconnectedListener(this);
        OpenNI::removeDeviceStateChangedListener(this);

        OpenNI::shutdown();
    }

public:
    virtual void onDeviceStateChanged (const DeviceInfo* pInfo, DeviceState state)
    {
        ntk_info("OpenNI2: Device \"%s\" state changed to %d.\n", pInfo->getUri(), state);
    }

    virtual void onDeviceConnected (const DeviceInfo* info)
    {
        ntk_info("OpenNI2: Device \"%s\" connected.\n", info->getUri());

        QMutexLocker _(&mutex);

        infos.insert(SensorInfo(*info));
    }

    virtual void onDeviceDisconnected (const DeviceInfo* info)
    {
        ntk_info("OpenNI2: Device \"%s\" disconnected.\n", info->getUri());

        QMutexLocker _(&mutex);

        infos.erase(infos.find(SensorInfo(*info)));
    }

    void
    getSensorInfos (SensorInfos& sensorInfos) const
    {
        const QMutexLocker _(&mutex);
        SensorInfos ret(infos.begin(), infos.end());
        std::swap(ret, sensorInfos);
    }

    bool isReady () const { return ready; }
    QString getLastError () const
    {
        return OpenNI::getExtendedError();
    }

private:
    Openni2Driver* that;
    mutable QMutex mutex;
    bool ready;
    std::set<SensorInfo> infos;
};

FWD_IMPL_1_CONST(void, Openni2Driver, getSensorInfos, SensorInfos&)
FWD_IMPL_0_CONST(bool, Openni2Driver, isReady)
FWD_IMPL_0_CONST(QString, Openni2Driver, getLastError)

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

namespace openni {

inline bool operator == (const VideoStream& lhs, const VideoStream& rhs)
{
    return lhs._getHandle() == rhs._getHandle();
}

}

//------------------------------------------------------------------------------

namespace ntk { namespace {

void debugStream (VideoStream& stream)
{
    VideoFrameRef frame;

    stream.readFrame(&frame);

    DepthPixel* pDepth;
    RGB888Pixel* pColor;

    int middleIndex = (frame.getHeight()+1) * frame.getWidth() / 2;

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

bool
haveEqualSize (const VideoMode& first, const VideoMode& second)
{
    const bool sameWidth  = first.getResolutionX() == second.getResolutionX();
    const bool sameHeight = first.getResolutionY() == second.getResolutionY();

    return sameWidth && sameHeight;
}

bool
haveEqualSize (const cv::Mat& matrix, VideoFrameRef frame)
{
    const bool sameWidth  = matrix.cols == frame.getWidth();
    const bool sameHeight = matrix.rows == frame.getHeight();

    return sameWidth && sameHeight;
}

const char*
getPixelFormatName (PixelFormat format)
{
    switch (format)
    {
    case openni::PIXEL_FORMAT_DEPTH_1_MM:
        return "1 mm";
    case openni::PIXEL_FORMAT_DEPTH_100_UM:
        return "100 um";
    case openni::PIXEL_FORMAT_SHIFT_9_2:
        return "Shifts 9.2";
    case openni::PIXEL_FORMAT_SHIFT_9_3:
        return "Shifts 9.3";
    case openni::PIXEL_FORMAT_RGB888:
        return "RGB 888";
    case openni::PIXEL_FORMAT_YUV422:
        return "YUV 422";
    case openni::PIXEL_FORMAT_GRAY8:
        return "Grayscale 8-bit";
    case openni::PIXEL_FORMAT_GRAY16:
        return "Grayscale 16-bit";
    case openni::PIXEL_FORMAT_JPEG:
        return "JPEG";
    default:
        return "Unknown";
    }
}

bool
decode16BitDepthFrame (RGBDImage &image, VideoFrameRef frame)
{
    if (!haveEqualSize(image.rawRgbRef(), frame))
    {
        ntk_error("OpenNI2: Bad depth frame size.\n");
        return false;
    }

    const DepthPixel* framePixels = reinterpret_cast<const DepthPixel*>(frame.getData());
    const int         frameSize   = frame.getDataSize() / sizeof(DepthPixel);
    quint16*          imagePixels = image.rawDepth16bitsRef().ptr<quint16>();

    std::copy(framePixels, framePixels + frameSize, imagePixels);

    return true;
}

bool
decodeDepthFrame_1_MM (RGBDImage& image, VideoFrameRef frame)
{
    // FIXME: Is there any additional processing needed here?
    return decode16BitDepthFrame(image, frame);
}

bool
decodeDepthFrame_100_UM (RGBDImage& image, VideoFrameRef frame)
{
    // FIXME: Is there any additional processing needed here?
    return decode16BitDepthFrame(image, frame);
}

bool
decodeDepthFrame (RGBDImage& image, VideoFrameRef frame)
{
    const VideoMode& mode = frame.getVideoMode();
    const PixelFormat& format = mode.getPixelFormat();

    switch (format)
    {
        case PIXEL_FORMAT_DEPTH_1_MM:   return decodeDepthFrame_1_MM(image, frame);
        case PIXEL_FORMAT_DEPTH_100_UM: return decodeDepthFrame_100_UM(image, frame);

        case PIXEL_FORMAT_SHIFT_9_2:    // FIXME: Implement.
        case PIXEL_FORMAT_SHIFT_9_3:    // FIXME: Implement.

    default:
        ntk_error("OpenNI2: Unhandled depth frame format: %s.\n", getPixelFormatName(format));
    }

    return false;
}

bool
decodeColorFrame_RGB888 (RGBDImage& image, VideoFrameRef frame)
{
    if(!haveEqualSize(image.rawRgbRef(), frame))
    {
        ntk_error("OpenNI2: Bad color frame size.\n");

        return false;
    }

 // const RGB888Pixel* framePixels = reinterpret_cast<const RGB888Pixel*>(frame.getData());
    const quint8*      framePixels = reinterpret_cast<const quint8*     >(frame.getData());
    const int          frameSize   = frame.getDataSize() / sizeof(RGB888Pixel) * 3;
    quint8*            imagePixels = image.rawRgbRef().ptr<quint8>();

    // FIXME: RGB or BGR?
    std::copy(framePixels, framePixels + frameSize, imagePixels);

    return true;
}

bool
decodeColorFrame (RGBDImage& image, VideoFrameRef frame)
{
    const VideoMode& mode = frame.getVideoMode();
    const PixelFormat& format = mode.getPixelFormat();

    switch (format)
    {
        case PIXEL_FORMAT_RGB888: return decodeColorFrame_RGB888(image, frame);
        case PIXEL_FORMAT_YUV422: // FIXME: Implement.
        case PIXEL_FORMAT_GRAY8 : // FIXME: Implement.
        case PIXEL_FORMAT_GRAY16: // FIXME: Implement.
        case PIXEL_FORMAT_JPEG  : // FIXME: Implement.

    default:
        ntk_error("OpenNI2: Unhandled color frame format: %s.\n", getPixelFormatName(format));
    }

    return false;
}

void
prepareFrameImage (RGBDImage& image, int width, int height, RGBDCalibrationConstPtr calibration)
{
    const cv::Size size(width, height);

    image.rawDepth16bitsRef() = cv::Mat1w(size);
    image.rawRgbRef() = cv::Mat3b(size);

    // FIXME: Fix image calibration estimation and enable this.
    // image.setCalibration(calibration);
}

} }

//------------------------------------------------------------------------------

namespace ntk {

struct Openni2Grabber::Impl
{
    Impl (Openni2Grabber* that_, Openni2Driver& driver_, QString uri_)
        : that(that_)
        , driver(driver_)
        , uri(uri_)
    {
        color.listener.that = this;
        depth.listener.that = this;
    }

    ~Impl ()
    {

    }

    void
    onNewFrame (VideoStream& stream)
    {
        VideoFrameRef frame;

        stream.readFrame(&frame);

        // debugStream(stream);

        if (depth.stream == stream)
        {
            ntk_dbg(2) << "OpenNI2: Depth frame available.\n";

            QMutexLocker _(&mutex);

            depth.frame = frame;
            depth.ready = true;
        }

        if (color.stream == stream)
        {
            ntk_dbg(2) << "OpenNI2: Color frame available.\n";

            QMutexLocker _(&mutex);

            color.frame = frame;
            color.ready = true;
        }

        onPartialFrame();
    }

    void
    onPartialFrame ()
    {
        if (!color.ready || !depth.ready)
            return; // Stubbornly refuse to expose incomplete frames.

        image.setTimestamp(that->getCurrentTimestamp());

        VideoFrameRef depthFrame;
        VideoFrameRef colorFrame;

        {
            QMutexLocker _(&mutex);

            depthFrame = depth.frame;
            colorFrame = color.frame;

            depth.ready = false;
            color.ready = false;
        }

        if (!decodeDepthFrame(image, depthFrame))
            return;

        if (!decodeColorFrame(image, colorFrame))
            return;

        {
            QMutexLocker _(&mutex);

            // FIXME: Some m_rgbd_image locking should be necessary here. Check that.
            image.swap(that->m_rgbd_image);
        }

        that->advertiseNewFrame();
    }

    RGBDCalibrationConstPtr
    estimateCalibration (int width, int height) const
    {
        const cv::Size size(width, height);

        RGBDCalibration* ret = new RGBDCalibration;

        ret->setRawRgbSize(size);
        ret->setRgbSize(size);
        ret->raw_depth_size = size;
        ret->depth_size = size;

        // FIXME: Set missing calibration data.

        return ret;
    }

    Openni2Grabber* that;

    Openni2Driver& driver;
    QString uri;

    RGBDImage image;

    int subsampling;
    bool highRes;
    bool mirrored;
    bool customBayerDecoding;
    bool hardwareRegistration;

    struct Channel
    {
        Channel ()
            : ready(false)
        {

        }

        struct FrameListener : VideoStream::NewFrameListener
        {
            virtual void onNewFrame (VideoStream& stream)
            {
                that->onNewFrame(stream);
            }

            Impl* that;
        };

        FrameListener listener;
        bool             ready;
        VideoFrameRef    frame;
        VideoStream     stream;
    };

    Channel depth;
    Channel color;

    Device device;

    QMutex mutex;
};

Openni2Grabber::Openni2Grabber (Openni2Driver& driver, QString uri)
    : impl(new Impl(this, driver, uri))
{

}

Openni2Grabber::~Openni2Grabber ()
{
    delete impl;
}

bool
Openni2Grabber::connectToDevice ()
{
    if (m_connected)
        return true;

    QMutexLocker _(&impl->mutex);
    ntk_info("OpenNI2: Opening: %s\n", impl->uri.toUtf8().constData());

    Openni2Driver::SensorInfos sensorInfos;
    impl->driver.getSensorInfos(sensorInfos);
    ntk_info("OpenNI2: Number of devices: %d\n", sensorInfos.size());
    ntk_dbg_print(sensorInfos.size(), 1);

    Status status = STATUS_OK;

    if (impl->uri.isEmpty())
        status = impl->device.open(ANY_DEVICE);
    else
        status = impl->device.open(impl->uri.toUtf8().constData());

    if (STATUS_OK != status)
    {
        ntk_error("OpenNI: %s\n", OpenNI::getExtendedError());
        return false;
    }

    if (NULL != impl->device.getSensorInfo(SENSOR_DEPTH))
    {
        status = impl->depth.stream.create(impl->device, SENSOR_DEPTH);
        if (STATUS_OK != status)
        {
            ntk_error("OpenNI2: Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
            return false;
        }
    }

    if (NULL != impl->device.getSensorInfo(SENSOR_COLOR))
    {
        status = impl->color.stream.create(impl->device, SENSOR_COLOR);
        if (STATUS_OK != status)
        {
            ntk_error("OpenNI2: Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
            return false;
        }
    }

    // FIXME: SENSOR_IR is also available. Expose it.

    // impl->device.setDepthColorSyncEnabled(true);

    m_connected = true;
    return true;
}

bool
Openni2Grabber::disconnectFromDevice ()
{
    if (!m_connected)
        return true;

    m_connected = false;

    impl->color.stream.destroy();
    impl->depth.stream.destroy();

    impl->device.close();

    return true;
}

void
Openni2Grabber::setIRMode (bool ir)
{
    // FIXME: Implement.
}

void
Openni2Grabber::setSubsamplingFactor (int factor)
{
    // FIXME: Implement.
}

void
Openni2Grabber::setHighRgbResolution(bool hr)
{
    impl->highRes = hr;
}

void
Openni2Grabber::setMirrored(bool m)
{
    impl->mirrored = m;
}

void
Openni2Grabber::setCustomBayerDecoding(bool enable)
{
    impl->customBayerDecoding = enable;
}

void
Openni2Grabber::setUseHardwareRegistration(bool enable)
{
    impl->hardwareRegistration = enable;
}

void
Openni2Grabber::run ()
{
    if (!m_connected)
        ntk_error("OpenNI2: Cannot start grabbing: Device not connected.");

    Status status = STATUS_OK;

    // FIXME: Handle differing depth and color frame sizes.
    if (!haveEqualSize(impl->depth.stream.getVideoMode(), impl->color.stream.getVideoMode()))
        ntk_error("OpenNI2: Cannot start grabbing: Incompatible depth and stream sizes.");

    const int frameWidth  = impl->depth.stream.getVideoMode().getResolutionX();
    const int frameHeight = impl->depth.stream.getVideoMode().getResolutionY();

    RGBDCalibrationConstPtr calibration = impl->estimateCalibration(frameWidth, frameHeight);

    prepareFrameImage( impl->image, frameWidth, frameHeight, calibration);
    prepareFrameImage(m_rgbd_image, frameWidth, frameHeight, calibration);

    impl->depth.stream.addNewFrameListener(&impl->depth.listener);
    impl->depth.stream.start();
    if (STATUS_OK  != status)
    {
        ntk_error("OpenNI2: Cannot start grabbing: Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        return;
    }

    impl->color.stream.addNewFrameListener(&impl->color.listener);
    impl->color.stream.start();
    if (STATUS_OK  != status)
    {
        ntk_error("OpenNI2: Cannot start grabbing: Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
        return;
    }

    while (!threadShouldExit())
        msleep(500);

    impl->color.stream.stop();
    impl->depth.stream.stop();

    impl->color.stream.removeNewFrameListener(&impl->color.listener);
    impl->depth.stream.removeNewFrameListener(&impl->depth.listener);
}

}
