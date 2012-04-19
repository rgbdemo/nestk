#include "kin4win_grabber.h"

#include "NuiApi.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
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

typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;

typedef OLECHAR* WinStr;
typedef HRESULT  WinRet;
typedef HANDLE   WinHandle;

void alert (WinStr txt)
{
    MessageBoxW(0, txt, L"Kinect", MB_OK | MB_ICONHAND);
}

void debug (WinStr txt)
{
    OutputDebugStringW(txt);
}

}

namespace Messages
{
#define MSG(Var, Txt)         \
    static WinStr Var = Txt;

    MSG(initError         , L"Failed to initialize sensor.")
    MSG(depthStreamError  , L"Could not open depth stream.")
    MSG(colorStreamError  , L"Could not open color stream.")
    MSG(creationError     , L"Failed to acquire sensor."   )
    MSG(inUseError        , L"Sensor already in use."      )

#undef MSG
}

namespace ntk {

// FIXME: NuiSetDeviceStatusCallback( &CSkeletalViewerApp::Nui_StatusProcThunk, that);

class Nui
{
public:
    static const int defaultFrameWidth  = 640;
    static const int defaultFrameHeight = 480;

    Nui (Kin4WinGrabber* that)
    : that(that)
    , dirtyDepth(true)
    , dirtyColor(true)
    , sensor(0)
    , name(0)
    {

    }

    static void statusChanged (WinRet result, WinStr instanceName, WinStr uniqueDeviceName, void* data)
    {
        reinterpret_cast<Nui*>(data)->statusChanged(result, instanceName, uniqueDeviceName);
    }

    bool isSameInstance (WinStr instanceName) const
    {
        0 != name && 0 == wcscmp(name, instanceName);
    }

    void statusChanged (WinRet result, WinStr instanceName, WinStr uniqueDeviceName)
    {
        // FIXME: Update UI.

        if(!SUCCEEDED(result))
        {
            if (isSameInstance(instanceName))
            {
                unInit();
                zero();
            }
            return;
        }

        if (S_OK == result)
        {
            if (isSameInstance(instanceName))
                init(instanceName);
            else if (!sensor)
                init();
        }
    }

    WinRet init (WinStr instanceName)
    {
        if (0 == instanceName)
        {
            alert(Messages::creationError);
            return E_FAIL;
        }

        WinRet ret = NuiCreateSensorById(instanceName, &sensor);
    
        if (FAILED(ret))
        {
            alert(Messages::creationError);
            return ret;
        }

        SysFreeString(name);

        name = sensor->NuiDeviceConnectionId();

        return init();
    }

    static DWORD WINAPI processThread (void* data)
    {
        Nui* that = reinterpret_cast<Nui*>(data);

        return that->processThread();
    }

    WinRet init ()
    {
        WinRet ret;
        bool result;

        if (0 == sensor)
        {
            WinRet ret = NuiCreateSensorByIndex(0, &sensor);

            if (FAILED(ret))
                return ret;

            SysFreeString(name);

            name = sensor->NuiDeviceConnectionId();
        }
        
        nextDepthFrameEvent = CreateEvent(0, TRUE, FALSE, 0);
        nextColorFrameEvent = CreateEvent(0, TRUE, FALSE, 0);
   
        ret = sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
 
        if (FAILED(ret))
        {
            if (E_NUI_DEVICE_IN_USE == ret)
                alert(Messages::inUseError);
            else
                alert(Messages::initError);

            return ret;
        }

        ret = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, nextColorFrameEvent, &colorStreamHandle);

        if (FAILED(ret))
        {
            alert(Messages::colorStreamError);
            return ret;
        }

        ret = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2, nextDepthFrameEvent, &depthStreamHandle);

        if (FAILED(ret))
        {
            alert(Messages::depthStreamError);
            return ret;
        }

        stopProcessingEvent = CreateEvent(0, FALSE, FALSE, 0);
        processingThreadHandle = CreateThread(NULL, 0, processThread, this, 0, NULL);

        return ret;
    }

    DWORD processThread ()
    {
        const int numEvents = 3;
        WinHandle events[numEvents] = { stopProcessingEvent, nextDepthFrameEvent, nextColorFrameEvent };
        int eventIndex = -1;
        DWORD t;

        bool continueProcessing = true;

        while (continueProcessing)
        {
            // Wait for all of the events.
            eventIndex = WaitForMultipleObjects(numEvents, events, FALSE, 100);

            // Process the next event.
            switch (eventIndex)
            {
                case WAIT_TIMEOUT:
                    continue;

                case WAIT_OBJECT_0:
                    continueProcessing = false;
                    continue;

                case WAIT_OBJECT_0 + 1:
                    nextDepthFrame();
                    break;

                case WAIT_OBJECT_0 + 2:
                    nextColorFrame();
                    break;
            }
        }

        return 0;
    }

    void nextColorFrame ()
    {
        NUI_IMAGE_FRAME colorFrame;

        WinRet ret = sensor->NuiImageStreamGetNextFrame(colorStreamHandle, 0, &colorFrame);

        if (FAILED(ret))
            return;

        INuiFrameTexture* texture = colorFrame.pFrameTexture;
        NUI_LOCKED_RECT lockedRect;
        texture->LockRect(0, &lockedRect, NULL, 0);
        if (lockedRect.Pitch != 0)
        {
            NUI_SURFACE_DESC surfaceDesc;
            texture->GetLevelDesc(0, &surfaceDesc);
            const int width  = surfaceDesc.Width;
            const int height = surfaceDesc.Height;

            ntk_assert( width == that->m_current_image.rawRgb().cols, "Bad width");
            ntk_assert(height == that->m_current_image.rawRgb().rows, "Bad height");

            uint8_t* buf = static_cast<uint8_t*>(lockedRect.pBits);
            // size_t  size = size_t(lockedRect.size);

            cvtColor(cv::Mat4b(height, width, (cv::Vec4b*)buf), that->m_current_image.rawRgbRef(), CV_RGBA2RGB);

            // std::copy(buf, buf + 3 * width * height, (uint8_t*) that->m_current_image.rawRgbRef().ptr());
            // cvtColor(that->m_current_image.rawRgb(), that->m_current_image.rawRgbRef(), CV_RGB2BGR);

            // m_pDrawColor->Draw( static_cast<BYTE *>(LockedRect.pBits), LockedRect.size );
        }
        else
        {
            debug(L"Buffer length of received texture is bogus\r\n");
        }

        texture->UnlockRect(0);

        sensor->NuiImageStreamReleaseFrame(colorStreamHandle, &colorFrame);

        dirtyColor = false;
    }

    void nextDepthFrame ()
    {
        NUI_IMAGE_FRAME depthFrame;

        WinRet ret = sensor->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame);

        if (FAILED(ret))
            return;

        INuiFrameTexture* texture = depthFrame.pFrameTexture;
        NUI_LOCKED_RECT lockedRect;
        texture->LockRect(0, &lockedRect, NULL, 0);
        if (0 != lockedRect.Pitch)
        {
            NUI_SURFACE_DESC surfaceDesc;
            texture->GetLevelDesc(0, &surfaceDesc);
            const int width  = surfaceDesc.Width;
            const int height = surfaceDesc.Height;

            uint16_t* buf = reinterpret_cast<uint16_t*>(lockedRect.pBits);

            ntk_assert(width  == that->m_current_image.rawDepth().cols, "Bad width");
            ntk_assert(height == that->m_current_image.rawDepth().rows, "Bad height");

            float* depth_buf = that->m_current_image.rawDepthRef().ptr<float>();
            for (int i = 0; i < width * height; ++i)
                *depth_buf++ = float(NuiDepthPixelToDepth(*buf++)) / 1000.f;

            //DWORD frameWidth, frameHeight;
        
            //NuiImageResolutionToSize(imageFrame.eResolution, frameWidth, frameHeight);
        
            //// draw the bits to the bitmap
            //RGBQUAD * rgbrun = m_rgbWk;
            //USHORT * pBufferRun = (USHORT *)LockedRect.pBits;

            //// end pixel is start + width*height - 1
            //USHORT * pBufferEnd = pBufferRun + (frameWidth * frameHeight);

            //assert( frameWidth * frameHeight <= ARRAYSIZE(m_rgbWk) );

            //while ( pBufferRun < pBufferEnd )
            //{
            //    *rgbrun = Nui_ShortToQuad_Depth( *pBufferRun );
            //    ++pBufferRun;
            //    ++rgbrun;
            //}

            // m_pDrawDepth->Draw( (BYTE*) m_rgbWk, frameWidth * frameHeight * 4 );
        }
        else
        {
            debug(L"Buffer length of received texture is bogus\r\n");
        }

        texture->UnlockRect(0);

        sensor->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame);

        dirtyDepth = false;
    }

    void unInit( )
    {
        // Stop the processing thread.
        if (0 != stopProcessingEvent)
        {
            // Signal the thread.
            SetEvent(stopProcessingEvent);

            // Wait for thread to stop.
            if (0 != processingThreadHandle)
            {
                WaitForSingleObject(processingThreadHandle, INFINITE);
                CloseHandle(processingThreadHandle);
            }

            CloseHandle(stopProcessingEvent);
        }

        if (0 != sensor)
            sensor->NuiShutdown();

        if (0 != nextDepthFrameEvent && INVALID_HANDLE_VALUE != nextDepthFrameEvent)
        {
            CloseHandle(nextDepthFrameEvent);
            nextDepthFrameEvent = 0;
        }
        if (0 != nextColorFrameEvent && INVALID_HANDLE_VALUE !=  nextColorFrameEvent)
        {
            CloseHandle(nextColorFrameEvent);
            nextColorFrameEvent = 0;
        }

        if (0 != sensor)
        {
            sensor->Release();
            sensor = 0;
        }
    }

    void zero()
    {
        if (0 != sensor)
        {
            sensor->Release();
            sensor = 0;
        }
        nextDepthFrameEvent = 0;
        nextColorFrameEvent = 0;
        depthStreamHandle = 0;
        colorStreamHandle = 0;
        processingThreadHandle = 0;
        stopProcessingEvent = 0;
    }

    Kin4WinGrabber* that;

    WinStr name;

    INuiSensor* sensor;

    WinHandle nextDepthFrameEvent;
    WinHandle     depthStreamHandle;
    bool     dirtyDepth;

    WinHandle nextColorFrameEvent;
    WinHandle     colorStreamHandle;
    bool     dirtyColor;

    WinHandle stopProcessingEvent;
    WinHandle     processingThreadHandle;
};

}

//------------------------------------------------------------------------------

namespace ntk
{

Kin4WinGrabber :: Kin4WinGrabber(Kin4WinDriver& driver, int camera_id) :
    nui(new Nui(this)),
    m_driver(driver),
    m_camera_id(camera_id)
{

}

Kin4WinGrabber :: Kin4WinGrabber(Kin4WinDriver& driver, const std::string& camera_serial) :
    nui(new Nui(this)),
    m_driver(driver),
    m_camera_id(-1),
    m_camera_serial(camera_serial)
{
    //for (size_t i = 0; i < driver.numDevices(); ++i)
    //{
    //    if (driver.deviceInfo(i).serial == camera_serial)
    //    {
    //        m_camera_id = i;
    //        break;
    //    }
    //}

    if (m_camera_id < 0)
    {
        ntk_throw_exception("Could not find any device with serial " + camera_serial);
    }
}

bool Kin4WinGrabber :: connectToDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_dbg(1) << format("[Kinect %x] connecting", this);

    //nui->init();

    return false;
}

bool Kin4WinGrabber :: disconnectFromDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_dbg(1) << format("[Kinect %x] disconnecting", this);

    nui->unInit();

    return false;
}

void Kin4WinGrabber :: estimateCalibration()
{

}

void Kin4WinGrabber :: run()
{
    m_should_exit = false;

    //m_current_image.setCalibration(m_calib_data);
    //m_rgbd_image.setCalibration(m_calib_data);

    m_rgbd_image.rawRgbRef()   = Mat3b(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    m_rgbd_image.rawDepthRef() = Mat1f(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    //m_rgbd_image.rawIntensityRef() = Mat1f(defaultFrameHeight, defaultFrameWidth);

    m_current_image.rawRgbRef()   = Mat3b(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    m_current_image.rawDepthRef() = Mat1f(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    //m_current_image.rawIntensityRef() = Mat1f(defaultFrameHeight, defaultFrameWidth);

    nui->init();

    int64 last_grab_time = 0;

    while (!m_should_exit)
    {
        waitForNewEvent();

        while (nui->dirtyDepth || nui->dirtyColor)
        {
            Sleep(1);
        }

        // m_current_image.rawDepth().copyTo(m_current_image.rawAmplitudeRef());
        // m_current_image.rawDepth().copyTo(m_current_image.rawIntensityRef());

        {
            int64 grab_time = ntk::Time::getMillisecondCounter();
            ntk_dbg_print(grab_time - last_grab_time, 2);
            last_grab_time = grab_time;
            QWriteLocker locker(&m_lock);

            m_current_image.swap(m_rgbd_image);

            nui->dirtyDepth = true;
            nui->dirtyColor = true;
        }

        advertiseNewFrame();
    }
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
