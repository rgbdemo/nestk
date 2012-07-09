#include "kin4win_grabber.h"

#include <Ole2.h>
#include "NuiApi.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/gui/image_window.h>

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
typedef NUI_IMAGE_RESOLUTION NuiResolution;

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
    , depthResolution(NUI_IMAGE_RESOLUTION_640x480)
    , dirtyColor(true)
    , colorResolution(NUI_IMAGE_RESOLUTION_640x480)
    , sensor(0)
    , name(0)
    , colorCoordinates(0)
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
        colorCoordinates = new long[getDepthHeight()*getDepthWidth()*2];
        mappedDepthTmp = new float[getDepthHeight()*getDepthWidth()];

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

        ret = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, colorResolution, 0, 2, nextColorFrameEvent, &colorStreamHandle);

        if (FAILED(ret))
        {
            alert(Messages::colorStreamError);
            return ret;
        }

        ret = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, depthResolution, 0, 2, nextDepthFrameEvent, &depthStreamHandle);

        NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);

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

            if (that->m_near_mode_changed)
            {
                ntk_dbg_print(that->m_near_mode_changed, 0);
                ntk_dbg_print(that->m_near_mode, 0);
                if (that->m_near_mode)
                    NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
                else
                    NuiImageStreamSetImageFrameFlags(depthStreamHandle, 0);
                that->m_near_mode_changed = false;
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

            {
                QWriteLocker locker(&that->m_lock);
                cvtColor(cv::Mat4b(height, width, (cv::Vec4b*)buf), that->m_current_image.rawRgbRef(), CV_RGBA2RGB);
            }

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

            {
                QWriteLocker locker(&that->m_lock);
                float* depth_buf = that->m_current_image.rawDepthRef().ptr<float>();
                mapDepthFrameToRgbFrame(buf, depth_buf);
            }

#if 0
            for (int i = 0; i < width * height; ++i)
                *depth_buf++ = float(NuiDepthPixelToDepth(*buf++)) / 1000.f;
#endif

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

        delete [] colorCoordinates; colorCoordinates = 0;
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

    int getResolutionWidth (NuiResolution nuiRes) const
    {
        switch (nuiRes)
        {
            case NUI_IMAGE_RESOLUTION_INVALID : return -1 ;

            case NUI_IMAGE_RESOLUTION_80x60   : return 80 ;
            case NUI_IMAGE_RESOLUTION_320x240 : return 320;
            case NUI_IMAGE_RESOLUTION_640x480 : return 640;
            case NUI_IMAGE_RESOLUTION_1280x960: return 1280;

            default: return 0;
        }
    }

    int getResolutionHeight (NuiResolution nuiRes) const
    {
        switch (nuiRes)
        {
            case NUI_IMAGE_RESOLUTION_INVALID : return -1 ;

            case NUI_IMAGE_RESOLUTION_80x60   : return 60 ;
            case NUI_IMAGE_RESOLUTION_320x240 : return 240;
            case NUI_IMAGE_RESOLUTION_640x480 : return 480;
            case NUI_IMAGE_RESOLUTION_1280x960: return 960;

            default: return 0;
        }
    }

    int getColorWidth () const
    {
        return getResolutionWidth(colorResolution);
    }

    int getColorHeight () const
    {
        return getResolutionHeight(colorResolution);            
    }

    int getDepthWidth () const
    {
        return getResolutionWidth(depthResolution);
    }

    int getDepthHeight () const
    {
        return getResolutionHeight(depthResolution);            
    }

    // FIXME: Untested.
    cv::Point3f depthToRealWorld (cv::Point3f p)
    {
        const int w = getDepthWidth();
        const int h = getDepthHeight();

        Vector4 v = NuiTransformDepthImageToSkeleton(
            LONG(double(w - p.x - 1) / double(w)),
            LONG(double(p.y) / double(h)),
            USHORT(p.z * 1000) << 3,
            depthResolution
        );

        return cv::Point3f(v.x, v.y, v.z);
    }

    cv::Point3f realWorldToDepth (const cv::Point3f& p)
    {
        const Vector4 v = { p.x, p.y, p.z, 1.f };
        LONG x = 0;
        LONG y = 0;
        USHORT d = 0;

        // FIXME: This should be resolution-dependent.
        NuiTransformSkeletonToDepthImage(v, &x, &y, &d, depthResolution);

        d >>= 3;

        return cv::Point3f(x, y, float(d) / 1000.f);
    }

    void postprocessDepthData (float* depth_values)
    {
        static const int neighbors[6][2] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, 1} };
        const size_t depth_width = getDepthWidth();
        const size_t depth_height = getDepthHeight();

        // Temporary buffer.
        std::copy(depth_values, depth_values + depth_width*depth_height, mappedDepthTmp);

        for (int ref_r = 0; ref_r < depth_height; ++ref_r)
        for (int ref_c = 0; ref_c < depth_width; ++ref_c)
        {
            float& d = depth_values[ref_r*depth_width+ref_c];
            if (d > 1e-5) continue;

            for (int neighb = 0; neighb < 6; ++neighb)
            {
                int r = ref_r + neighbors[neighb][0];
                int c = ref_c + neighbors[neighb][1];

                if (c >= 0 && c < depth_width && r >= 0 && r < depth_height)
                {
                    float neighb_d = mappedDepthTmp[r*depth_width+c];
                    if (neighb_d > 1e-5)
                    {
                        d = mappedDepthTmp[r*depth_width+c];
                        break;
                    }
                }
            }
        }
    }

    void mapDepthFrameToRgbFrame (uint16_t* depth_d16, float* depth_values)
    {
        const size_t depth_width = getDepthWidth();
        const size_t depth_height = getDepthHeight();

        sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
                    colorResolution,
                    depthResolution,
                    depth_width*depth_height,
                    depth_d16,
                    depth_width*depth_height*2,
                    colorCoordinates
                    );

        std::fill(depth_values, depth_values+depth_width*depth_height, 0.f);

        for (int i = 0; i < depth_width*depth_height; ++i)
        {
            int c = colorCoordinates[i*2];
            int r = colorCoordinates[i*2+1];

            uint16_t d = *depth_d16++;
            if (d == NUI_IMAGE_DEPTH_MINIMUM || d == NUI_IMAGE_DEPTH_MAXIMUM || d == NUI_IMAGE_DEPTH_NO_VALUE)
                continue;

            if (c >= 0 && c < depth_width && r >= 0 && r < depth_height)
            {
                
                float depth = float(NuiDepthPixelToDepth(d)) / 1000.f;
                int idx = r*depth_width+c;
                depth_values[idx] = depth;
            }
        }
    }


    Kin4WinGrabber* that;

    WinStr name;

    INuiSensor* sensor;

    long* colorCoordinates;
    float* mappedDepthTmp;

    NuiResolution depthResolution;
    WinHandle nextDepthFrameEvent;
    WinHandle     depthStreamHandle;
    bool     dirtyDepth;

    NuiResolution colorResolution;
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
    m_camera_id(camera_id),
    m_near_mode(true),
    m_near_mode_changed(true)
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

Kin4WinGrabber::~Kin4WinGrabber()
{
    // FIXME: RGBDGrabber calls stop in its own destructor,
    // destructors get called in derived-to-base order,
    // and stop is a virtual method.
    // Trouble.
    RGBDGrabber::stop();
}

bool Kin4WinGrabber :: connectToDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_dbg(1) << format("[Kinect %x] connecting", this);

    m_rgbd_image.rawRgbRef()   = Mat3b(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    m_rgbd_image.rawDepthRef() = Mat1f(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    //m_rgbd_image.rawIntensityRef() = Mat1f(defaultFrameHeight, defaultFrameWidth);

    m_current_image.rawRgbRef()   = Mat3b(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    m_current_image.rawDepthRef() = Mat1f(Nui::defaultFrameHeight, Nui::defaultFrameWidth);
    //m_current_image.rawIntensityRef() = Mat1f(defaultFrameHeight, defaultFrameWidth);

    // FIXME: This should be done at connectToDevice time.
    {
        WinRet ret = nui->init();

        if (FAILED(ret))
            return false;

        if (!m_calib_data)
        {
            estimateCalibration();
            m_current_image.setCalibration(m_calib_data);
            m_rgbd_image.setCalibration(m_calib_data);
        }
        m_connected = true;
    }

    return true;
}

bool Kin4WinGrabber :: disconnectFromDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_dbg(1) << format("[Kinect %x] disconnecting", this);

    nui->unInit();

    return false;
}

void Kin4WinGrabber::setNearMode(bool enable)
{
    m_near_mode = enable;
    m_near_mode_changed = true;
    ntk_dbg_print(m_near_mode, 0);
    ntk_dbg_print(m_near_mode_changed, 0);
}

void Kin4WinGrabber :: estimateCalibration()
{
    cv::Point3f p = nui->realWorldToDepth(cv::Point3f(0.f, 0.f, 1.f));

    double cx = p.x;
    double cy = p.y;

    p = nui->realWorldToDepth(cv::Point3f(1.f, 1.f, 1.f));

    double fx = (p.x - cx);
    double fy = -(p.y - cy);

    // These factors were estimated using chessboard calibration.
    // They seem to accurately correct the bias in object sizes output by
    // the default parameters.
    const double f_correction_factor = 528.0/570.34;
    // const double f_correction_factor = 1.0; // FIXME: what should it be for Kinect for Windows?
    fx *= f_correction_factor;
    fy *= f_correction_factor;

    printf("fx=%f\n", fx);
    printf("fy=%f\n", fy);
    printf("cx=%f\n", cx);
    printf("cy=%f\n", cy);
    ntk_dbg_print(fy, 1);
    ntk_dbg_print(cx, 1);
    ntk_dbg_print(cy, 1);

    // FIXME: this bias was not observed anymore in recent experiments.
    // const double cy_correction_factor = 267.0/240.0;
    const double cy_correction_factor = 1.0;
    cy *= cy_correction_factor;

    m_calib_data = new RGBDCalibration();

    int rgb_width  = nui->getColorWidth();
    int rgb_height = nui->getColorHeight();

    int depth_width  = nui->getDepthWidth();
    int depth_height = nui->getDepthHeight();

    m_calib_data->setRawRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->setRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->raw_depth_size = cv::Size(depth_width, depth_height);
    m_calib_data->depth_size = cv::Size(depth_width, depth_height);

    float width_ratio = float(rgb_width)/depth_width;
    float height_ratio = float(rgb_height)/depth_height;

    float rgb_fx = fx * width_ratio;
    // Pixels are square on a Kinect.
    // Image height gets cropped when going from 1280x1024 in 640x480.
    // The ratio remains 2.
    float rgb_fy = rgb_fx;
    float rgb_cx = cx * width_ratio;
    float rgb_cy = cy * width_ratio;

    m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->rgb_intrinsics);
    m_calib_data->rgb_intrinsics(0,0) = rgb_fx;
    m_calib_data->rgb_intrinsics(1,1) = rgb_fy;
    m_calib_data->rgb_intrinsics(0,2) = rgb_cx;
    m_calib_data->rgb_intrinsics(1,2) = rgb_cy;

    m_calib_data->rgb_distortion = Mat1d(1,5);
    m_calib_data->rgb_distortion = 0.;
    m_calib_data->zero_rgb_distortion = true;

    // After getAlternativeViewpoint, both camera have the same parameters.

    m_calib_data->depth_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->depth_intrinsics);
    m_calib_data->depth_intrinsics(0,0) = fx;
    m_calib_data->depth_intrinsics(1,1) = fy;
    m_calib_data->depth_intrinsics(0,2) = cx;
    m_calib_data->depth_intrinsics(1,2) = cy;

    m_calib_data->depth_distortion = Mat1d(1,5);
    m_calib_data->depth_distortion = 0.;
    m_calib_data->zero_depth_distortion = true;

    m_calib_data->R = Mat1d(3,3);
    setIdentity(m_calib_data->R);

    m_calib_data->T = Mat1d(3,1);
    m_calib_data->T = 0.;

    m_calib_data->depth_pose = new Pose3D();
    m_calib_data->depth_pose->setCameraParametersFromOpencv(m_calib_data->depth_intrinsics);

    m_calib_data->rgb_pose = new Pose3D();
    m_calib_data->rgb_pose->toRightCamera(m_calib_data->rgb_intrinsics,
                                          m_calib_data->R,
                                          m_calib_data->T);

    m_calib_data->camera_type = "kin4win";
}

void Kin4WinGrabber :: run()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    setThreadShouldExit(false);

    // setTargetFrameRate(20.f);

    int64 last_grab_time = 0;

    while (!threadShouldExit())
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

            m_current_image.setTimestamp(getCurrentTimestamp());
            {
                QWriteLocker locker(&m_lock);
                m_current_image.swap(m_rgbd_image);
                nui->postprocessDepthData(m_rgbd_image.rawDepthRef().ptr<float>());
                cv::flip(m_rgbd_image.rawDepth(), m_rgbd_image.rawDepthRef(), 1);
                cv::flip(m_rgbd_image.rawRgb(), m_rgbd_image.rawRgbRef(), 1);
            }

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

    INuiSensor* sensor = 0;
    // WinRet ret = NuiCreateSensorByIndex(0, &sensor);
    // if (SUCCEEDED(ret))
    {
        DeviceInfo device;
        device.serial = "unknown";
        device.camera_type = "kin4win";
        device.vendor = "Microsoft";
        devices_info.push_back(device);
    }

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
