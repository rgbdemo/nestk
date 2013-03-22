/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Tisserand <nicolas.tisserand@manctl.com>
 */

#include "kin4win_grabber.h"

#include <Ole2.h>
#include "NuiApi.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/gui/image_window.h>
#include <ntk/camera/calibration.h>

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
    MSG(nearModeError     , L"Could not set near mode.")

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
            ntk_error("Kinect for Windows: could not create a sensor handler.\n");
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
        mappedDepthTmp = new uint16_t[getDepthHeight()*getDepthWidth()];

        WinRet ret;
        bool result;

        if (0 == sensor)
        {
            WinRet ret = NuiCreateSensorByIndex(that->m_camera_id, &sensor);

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
            {
                alert(Messages::inUseError);
                ntk_error("Kinect for Windows already in use.\n");
            }
            else if (E_NUI_NOTGENUINE == ret)
            {
                ntk_error("Kinect for Windows is not genuine.\n");
            }
            else if (E_NUI_INSUFFICIENTBANDWIDTH == ret)
            {
                ntk_error("Insufficient bandwidth.\n");
            }
            else if (E_NUI_NOTSUPPORTED == ret)
            {
                ntk_error("Kinect for Windows device not supported.\n");
            }
            else if (E_NUI_NOTCONNECTED == ret)
            {
                ntk_error("Kinect for Windows is not connected.\n");
            }
            else if (E_NUI_NOTREADY == ret)
            {
                ntk_error("Kinect for Windows is not ready.\n");
            }
            else if (E_NUI_NOTPOWERED == ret)
            {
                ntk_error("Kinect for Windows is not powered.\n");
            }
            else if (E_NUI_DATABASE_NOT_FOUND == ret)
            {
                ntk_error("Kinect for Windows database not found.\n");
            }
            else if (E_NUI_DATABASE_VERSION_MISMATCH == ret)
            {
                ntk_error("Kinect for Windows database version mismatch.\n");
            }
            else
            {
                alert(Messages::initError);
                ntk_error("Kinect for Windows could not initialize.\n");
            }

            return ret;
        }

        ntk_info("Kinect for Windows: initialized successfully\n");

        ret = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, colorResolution, 0, 2, nextColorFrameEvent, &colorStreamHandle);

        if (FAILED(ret))
        {
            alert(Messages::colorStreamError);
            ntk_error("Kinect for Windows: could not open color stream\n");
            return ret;
        }

        ntk_info("Kinect for Windows: color stream opened.\n");

        ret = sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, depthResolution, 0, 2, nextDepthFrameEvent, &depthStreamHandle);

        if (FAILED(ret))
        {
            alert(Messages::depthStreamError);
            ntk_error("Kinect for Windows: could not open depth stream\n");
            return ret;
        }

        ntk_info("Kinect for Windows: depth stream opened.\n");

        ret = sensor->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);

        if (FAILED(ret))
        {
            ntk_warn("Kinect for Windows: could not set near mode\n");
            ntk_warn("Kinect for Windows: must be an Xbox Kinect.\n");
            // FIXME: for some unknown reason, the estimateCalibration technique does not work
            // with Kinect for Xbox. So use regular alignment in that case.
            that->m_align_depth_to_color = true;
            that->m_is_xbox_kinect = true;
            ret = SEVERITY_SUCCESS;
        }

        ntk_info("Kinect for Windows: flags set.\n");

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
                ntk_dbg_print(that->m_near_mode_changed, 1);
                ntk_dbg_print(that->m_near_mode, 1);
                if (that->m_near_mode)
                    sensor->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE);
                else
                    sensor->NuiImageStreamSetImageFrameFlags(depthStreamHandle, 0);
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

        BOOL nearModeOperational = false;
        INuiFrameTexture* texture = 0;
        ret = sensor->NuiImageFrameGetDepthImagePixelFrameTexture(depthStreamHandle, &depthFrame, &nearModeOperational, &texture);

        if (FAILED(ret))
            return;

        NUI_LOCKED_RECT lockedRect;
        texture->LockRect(0, &lockedRect, NULL, 0);
        if (0 != lockedRect.Pitch)
        {
            NUI_SURFACE_DESC surfaceDesc;
            texture->GetLevelDesc(0, &surfaceDesc);
            const int width  = surfaceDesc.Width;
            const int height = surfaceDesc.Height;

            NUI_DEPTH_IMAGE_PIXEL* extended_buf = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(lockedRect.pBits);

            ntk_assert(width  == that->m_current_image.rawDepth16bits().cols, "Bad width");
            ntk_assert(height == that->m_current_image.rawDepth16bits().rows, "Bad height");

            if (that->m_align_depth_to_color)
            {
                QWriteLocker locker(&that->m_lock);
                uint16_t* depth_buf = that->m_current_image.rawDepth16bitsRef().ptr<uint16_t>();
                mapDepthFrameToRgbFrame(extended_buf, depth_buf);
            }
            else
            {
                QWriteLocker locker(&that->m_lock);
                uint16_t* depth_buf = that->m_current_image.rawDepth16bitsRef().ptr<uint16_t>();
                cv::Vec2w* depth_to_color_coords = that->m_current_image.depthToRgbCoordsRef().ptr<cv::Vec2w>();
                extractDepthAndColorCoords (extended_buf, depth_buf, depth_to_color_coords);
            }
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

            DWORD ret = WAIT_TIMEOUT;

            // Wait for thread to stop.
            for (int i = 0; 0 != processingThreadHandle && ret == WAIT_TIMEOUT && i < 5; ++i)
            {
                ret = WaitForSingleObject(processingThreadHandle, 1000);
            }

            if (WAIT_TIMEOUT == ret)
                ntk_warn ("Could not properly shutdown Kinect for Windows grabber.\n");

            CloseHandle(processingThreadHandle);
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

    bool getAccelerometer(cv::Point3f& xyz) const
    {
        Vector4 v;
        WinRet ret = sensor->NuiAccelerometerGetCurrentReading(&v);
        if (FAILED(ret))
            return false;
        xyz.x = v.x;
        xyz.y = v.y;
        xyz.z = v.z;
        return true;
    }

    // FIXME: Untested.
    cv::Point3f depthToRealWorld (const cv::Point3f& p)
    {
        Vector4 v = NuiTransformDepthImageToSkeleton(
                    LONG(p.x),
                    LONG(p.y),
                    USHORT(p.z),
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

        return cv::Point3f(x, y, (float) d / 1000.f);
    }

    void postprocessDepthData (uint16_t* depth_values)
    {
        static const int neighbors[6][2] = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, 1} };
        const size_t depth_width = getDepthWidth();
        const size_t depth_height = getDepthHeight();

        // Temporary buffer.
        std::copy(depth_values, depth_values + depth_width*depth_height, mappedDepthTmp);

        for (int ref_r = 0; ref_r < depth_height; ++ref_r)
        for (int ref_c = 0; ref_c < depth_width; ++ref_c)
        {
            uint16_t& d = depth_values[ref_r*depth_width+ref_c];
            if (d > 0) continue;

            for (int neighb = 0; neighb < 6; ++neighb)
            {
                int r = ref_r + neighbors[neighb][0];
                int c = ref_c + neighbors[neighb][1];

                if (c >= 0 && c < depth_width && r >= 0 && r < depth_height)
                {
                    uint16_t neighb_d = mappedDepthTmp[r*depth_width+c];
                    if (neighb_d > 0)
                    {
                        d = mappedDepthTmp[r*depth_width+c];
                        break;
                    }
                }
            }
        }
    }

    void extractDepthAndColorCoords (NUI_DEPTH_IMAGE_PIXEL* extended_depth, uint16_t* depth_values, cv::Vec2w* color_coords)
    {
        const size_t depth_width = getDepthWidth();
        const size_t depth_height = getDepthHeight();
        const size_t rgb_width = getColorWidth();

        // First generate packed depth values from extended depth values, which include near pixels.
        for (int i = 0; i < depth_width*depth_height; ++i)
            depth_values[i] = extended_depth[i].depth << NUI_IMAGE_PLAYER_INDEX_SHIFT;

        sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
                    colorResolution,
                    depthResolution,
                    depth_width*depth_height,
                    depth_values, // depth_d16
                    depth_width*depth_height*2,
                    colorCoordinates
                    );

        for (int i = 0; i < depth_width*depth_height; ++i)
        {
            int c = colorCoordinates[i*2];
            int r = colorCoordinates[i*2+1];
            (*color_coords)[0] = r;
            (*color_coords)[1] = c;
            ++color_coords;

            // Restore unshifted value.
            depth_values[i] = extended_depth[i].depth;
        }
    }

    void mapDepthFrameToRgbFrame (NUI_DEPTH_IMAGE_PIXEL* extended_depth, uint16_t* depth_values)
    {
        const size_t depth_width = getDepthWidth();
        const size_t depth_height = getDepthHeight();
        const size_t rgb_width = getColorWidth();

        // First generate packed depth values from extended depth values, which include near pixels.
        for (int i = 0; i < depth_width*depth_height; ++i)
            depth_values[i] = extended_depth[i].depth << NUI_IMAGE_PLAYER_INDEX_SHIFT;

        sensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
                    colorResolution,
                    depthResolution,
                    depth_width*depth_height,
                    depth_values, // depth_d16
                    depth_width*depth_height*2,
                    colorCoordinates
                    );

        std::fill(depth_values, depth_values+depth_width*depth_height, 0);

        // FIXME: this is tricky. In our convention, we want a mapped depth image,
        // but keeping its original resolution. Kinect for Windows logically returns
        // a mapped depth image in high resolution if color is in high resolution.
        const float ratio = static_cast<float>(depth_width) / rgb_width;

        for (int i = 0; i < depth_width*depth_height; ++i)
        {
            int c = colorCoordinates[i*2] * ratio;
            int r = colorCoordinates[i*2+1] * ratio;
            // int r = i / depth_width;
            // int c = i % depth_width;

            // uint16_t d = *depth_d16++;
            uint16_t depth_in_mm = extended_depth[i].depth;

            if (depth_in_mm == 0)
                continue;

#if 0
            if (d < NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE
                    || d > NUI_IMAGE_DEPTH_MAXIMUM
                    || d == NUI_IMAGE_DEPTH_NO_VALUE
                    || d == NUI_IMAGE_DEPTH_TOO_FAR_VALUE
                    || d == NUI_IMAGE_DEPTH_UNKNOWN_VALUE)
                continue;
#endif

            if (c >= 0 && c < depth_width && r >= 0 && r < depth_height)
            {
                // uint16_t depth_in_mm = NuiDepthPixelToDepth(d);
                int idx = r*depth_width+c;
                depth_values[idx] = depth_in_mm;
            }
        }
    }


    Kin4WinGrabber* that;

    WinStr name;

    INuiSensor* sensor;

    long* colorCoordinates;
    uint16_t* mappedDepthTmp;

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
    m_near_mode_changed(true),
    m_high_resolution(false),
    m_align_depth_to_color(true),
    m_is_xbox_kinect(false)
{
}

Kin4WinGrabber :: Kin4WinGrabber(Kin4WinDriver& driver, const std::string& camera_serial) :
    nui(new Nui(this)),
    m_driver(driver),
    m_camera_id(-1),
    m_camera_serial(camera_serial),
    m_high_resolution(false),
    m_align_depth_to_color(true),
    m_is_xbox_kinect(false)
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
    disconnectFromDevice();
    RGBDGrabber::stop();
}

bool Kin4WinGrabber :: connectToDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_info("[Kinect %x] connecting\n", this);

    if (m_high_resolution)
        nui->colorResolution = NUI_IMAGE_RESOLUTION_1280x960;

    m_rgbd_image.rawRgbRef()   = Mat3b(nui->getColorHeight(), nui->getColorWidth());
    m_rgbd_image.rawDepth16bitsRef() = Mat1f(nui->getDepthHeight(), nui->getDepthWidth ());
    m_rgbd_image.depthToRgbCoordsRef() = Mat2w(nui->getDepthHeight(), nui->getDepthWidth ());
    //m_rgbd_image.rawIntensityRef() = Mat1f(defaultFrameHeight, defaultFrameWidth);

    m_current_image.rawRgbRef()   = Mat3b(nui->getColorHeight(), nui->getColorWidth());
    m_current_image.rawDepth16bitsRef() = Mat1f(nui->getDepthHeight(), nui->getDepthWidth ());
    m_current_image.depthToRgbCoordsRef() = Mat2w(nui->getDepthHeight(), nui->getDepthWidth ());
    //m_current_image.rawIntensityRef() = Mat1f(defaultFrameHeight, defaultFrameWidth);    

    // FIXME: This should be done at connectToDevice time.
    {
        WinRet ret = nui->init();

        if (FAILED(ret))
            return false;

        if (!m_calib_data)
        {
            estimateCalibration();
            updateCalibrationMinMaxDepth ();
            m_current_image.setCalibration(m_calib_data);
            m_rgbd_image.setCalibration(m_calib_data);
        }
        m_connected = true;
    }

    std::string serial = cv::format("0000000000000000-%d", m_camera_id);
    setCameraSerial(serial);

    m_rgbd_image.setCameraSerial(serial);
    m_current_image.setCameraSerial(serial);

    m_rgbd_image.setGrabberType(grabberType());
    m_current_image.setGrabberType(grabberType());

    if (false) // sample code using the accelerometer to identify each sensor.
    {
        cv::Point3f xyz;
        getAccelerometerXYZ(xyz);

        if (xyz.y < 0)
            setCameraSerial("0000000000000000");
        else
            setCameraSerial("0000000000000001");
    }

    return true;
}

bool Kin4WinGrabber :: disconnectFromDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);

    ntk_info("[Kinect %x] disconnecting\n", this);

    nui->unInit();

    return false;
}

bool Kin4WinGrabber :: getAccelerometerXYZ(cv::Point3f& xyz) const
{
    bool ok = nui->getAccelerometer(xyz);
    return ok;
}

void Kin4WinGrabber::setNearMode(bool enable)
{
    m_near_mode = enable;
    m_near_mode_changed = true;
    ntk_dbg_print(m_near_mode, 1);
    ntk_dbg_print(m_near_mode_changed, 1);
    updateCalibrationMinMaxDepth ();
}

void Kin4WinGrabber :: estimateCalibration()
{
    int rgb_width  = nui->getColorWidth();
    int rgb_height = nui->getColorHeight();

    int depth_width  = nui->getDepthWidth();
    int depth_height = nui->getDepthHeight();

    // Deduce depth focal from depth to world transforms.

    cv::Point3f p = nui->realWorldToDepth(cv::Point3f(0.f, 0.f, 1.f));

    double cx = p.x;
    double cy = p.y;

    p = nui->realWorldToDepth(cv::Point3f(1.f, 1.f, 1.f));

    double fx = (p.x - cx);
    double fy = -(p.y - cy);  

    ntk_dbg_print (fx, 1);
    ntk_dbg_print (fy, 1);

    m_calib_data = new RGBDCalibration();

    m_calib_data->setRawRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->setRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->raw_depth_size = cv::Size(depth_width, depth_height);
    m_calib_data->depth_size = cv::Size(depth_width, depth_height);

    float width_ratio = float(rgb_width)/depth_width;
    float height_ratio = float(rgb_height)/depth_height;

    const float correction_factor = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS
            / (NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f);
    // const float correction_factor = 1.0f;
    float rgb_fx = correction_factor * fx;
    // Pixels are square on a Kinect.
    // Image height gets cropped when going from 1280x1024 in 640x480.
    // The ratio remains 2.
    float rgb_fy = rgb_fx;
    float rgb_cx = cx * width_ratio;
    float rgb_cy = cy * width_ratio;

    // Initialize intrinsics.

    m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->rgb_intrinsics);
    m_calib_data->rgb_intrinsics(0,0) = rgb_fx;
    m_calib_data->rgb_intrinsics(1,1) = rgb_fy;
    m_calib_data->rgb_intrinsics(0,2) = rgb_cx;
    m_calib_data->rgb_intrinsics(1,2) = rgb_cy;

    m_calib_data->rgb_distortion = Mat1d(1,5);
    m_calib_data->rgb_distortion = 0.;
    m_calib_data->zero_rgb_distortion = true;

    m_calib_data->depth_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->depth_intrinsics);
    m_calib_data->depth_intrinsics(0,0) = fx;
    m_calib_data->depth_intrinsics(1,1) = fy;
    m_calib_data->depth_intrinsics(0,2) = cx;
    m_calib_data->depth_intrinsics(1,2) = cy;

    m_calib_data->depth_distortion = Mat1d(1,5);
    m_calib_data->depth_distortion = 0.;
    m_calib_data->zero_depth_distortion = true;

    // Estimate rgb intrinsics and stereo transform.
    {
        cv::RNG rng;
        // Estimate rgb calibration.
        std::vector< std::vector<Point3f> > model_points (30);
        std::vector< std::vector<Point2f> > rgb_points (30);
        std::vector< std::vector<Point2f> > depth_points (30);
        for (int i = 0; i < model_points.size(); ++i)
        {
            for (int k = 0; k < 50; ++k)
            {
                int r = rng(depth_height);
                int c = rng(depth_width);
                int depth = rng.uniform(NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE, NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE * 2);
                LONG color_c, color_r;
                HRESULT ret = nui->sensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
                            nui->colorResolution,
                            nui->depthResolution,
                            0, c, r, depth, &color_c, &color_r);
                if (!SUCCEEDED(ret))
                    continue;

                cv::Point3f world = nui->depthToRealWorld(cv::Point3f(c, r, depth));
                // ntk_dbg_print (cv::Point2f(c, r), 1);
                // ntk_dbg_print (cv::Point2f(color_c, color_r), 1);
                // ntk_dbg_print (world, 1);

                if (color_c >= 0 && color_c < rgb_width && color_r >= 0 && color_r < rgb_height)
                {
                    model_points[i].push_back(world);
                    rgb_points[i].push_back(cv::Point2f(color_c, color_r));
                    depth_points[i].push_back(cv::Point2f(c, r));
                }
            }
        }

        {
            std::vector<Mat> rvecs, tvecs;
            double reprojection_error = calibrateCamera(model_points, rgb_points, m_calib_data->rawRgbSize(),
                                                        m_calib_data->rgb_intrinsics, m_calib_data->rgb_distortion,
                                                        rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_ASPECT_RATIO | CV_CALIB_ZERO_TANGENT_DIST);
            ntk_dbg_print (reprojection_error, 1);
        }
        m_calib_data->rgb_distortion = 0.f;

        m_calib_data->T = 0.;
        setIdentity(m_calib_data->R);

        if (m_align_depth_to_color)
        {
            m_calib_data->rgb_intrinsics.copyTo(m_calib_data->depth_intrinsics);
        }
        else
        {
            cv::Mat E(3,3,CV_64F),F(3,3,CV_64F);
            stereoCalibrate(model_points,
                            rgb_points,
                            depth_points,
                            m_calib_data->rgb_intrinsics, m_calib_data->rgb_distortion,
                            m_calib_data->depth_intrinsics, m_calib_data->depth_distortion,
                            m_calib_data->raw_depth_size,
                            m_calib_data->R, m_calib_data->T, E, F,
                            TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, 1e-7),
                            CALIB_FIX_INTRINSIC);

            double stereo_reprojection_error = computeCalibrationError(F, rgb_points, depth_points);
            ntk_dbg_print (stereo_reprojection_error, 1);
        }
    }

    ntk_dbg_print (m_calib_data->rgb_intrinsics(0,0), 1);
    ntk_dbg_print (m_calib_data->rgb_intrinsics(1,1), 1);
    ntk_dbg_print (m_calib_data->rgb_intrinsics(0,2), 1);
    ntk_dbg_print (m_calib_data->rgb_intrinsics(1,2), 1);

    ntk_dbg_print (m_calib_data->depth_intrinsics(0,0), 1);
    ntk_dbg_print (m_calib_data->depth_intrinsics(1,1), 1);
    ntk_dbg_print (m_calib_data->depth_intrinsics(0,2), 1);
    ntk_dbg_print (m_calib_data->depth_intrinsics(1,2), 1);

    m_calib_data->depth_pose = new Pose3D();
    m_calib_data->depth_pose->setCameraParametersFromOpencv(m_calib_data->depth_intrinsics);

    m_calib_data->rgb_pose = new Pose3D();
    m_calib_data->rgb_pose->toRightCamera(m_calib_data->rgb_intrinsics,
                                          m_calib_data->R,
                                          m_calib_data->T);

    ntk_dbg_print (m_calib_data->rgb_pose->cvEulerRotation(), 1);
    ntk_dbg_print (m_calib_data->rgb_pose->cvTranslation(), 1);
}

void Kin4WinGrabber::updateCalibrationMinMaxDepth ()
{
    if (!m_calib_data)
        return;

    if (m_near_mode && !m_is_xbox_kinect)
    {
        m_calib_data->setMinDepthInMeters(0.4f);
        m_calib_data->setMaxDepthInMeters(2.0f);
    }
    else
    {
        m_calib_data->setMinDepthInMeters(0.4f);
        m_calib_data->setMaxDepthInMeters(5.0f);
    }
}

void Kin4WinGrabber :: run()
{
    setThreadShouldExit(false);

    // setTargetFrameRate(20.f);

    int64 last_grab_time = 0;

    while (!threadShouldExit())
    {
        waitForNewEvent(-1); // Use infinite timeout in order to honor sync mode.

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

            // FIXME: is this mutex really required?
            QMutexLocker ni_locker(&m_ni_mutex);

            m_current_image.setTimestamp(getCurrentTimestamp());
            {
                QWriteLocker locker(&m_lock);
                m_current_image.swap(m_rgbd_image);
                // FIXME: move this to rgbd_processor.
                // if (m_align_depth_to_color)
                    // nui->postprocessDepthData(m_rgbd_image.rawDepth16bitsRef().ptr<uint16_t>());
                cv::flip(m_rgbd_image.rawDepth16bits(), m_rgbd_image.rawDepth16bitsRef(), 1);
                cv::flip(m_rgbd_image.rawRgb(), m_rgbd_image.rawRgbRef(), 1);
            }

            nui->dirtyDepth = true;
            nui->dirtyColor = true;
        }

        advertiseNewFrame();
    }

}

} // ntk

bool
ntk::Kin4WinDriver::hasDll ()
{
    // Trigger Kinect SDK DLL loading by calling one of its functions.
    __try
    {
        int count;
        WinRet ret = NuiGetSensorCount(&count);
        return true;
    }
    __except(EXCEPTION_EXECUTE_HANDLER)
    {
        return false;
    }
}

ntk::Kin4WinDriver::Kin4WinDriver()
{
    ntk_info("Initializing Kin4Win driver\n");

    int kin4win_sensors_count;
    WinRet ret = NuiGetSensorCount(&kin4win_sensors_count);
    ntk_dbg(1) << cv::format("Number of Kinect for Windows devices: %d\n", kin4win_sensors_count);

    for (int i = 0; i < kin4win_sensors_count; ++i)
    {
        DeviceInfo device;
        device.serial = cv::format("0000000000000000-%d", i);
        device.camera_type = "kin4win";
        device.vendor = "Microsoft";
        devices_info.push_back(device);
    }

    // ntk_throw_exception(format("enumerating devices failed. Reason: %s", xnGetStatusString(status)));

    // ntk_dbg(1) << format("Found device: vendor %s name %s", description.strVendor, description.strName);

    // ntk_dbg(1) << cv::format("[Device %d] %s, %s, serial=%s",
}

ntk::Kin4WinDriver :: ~Kin4WinDriver()
{

}
