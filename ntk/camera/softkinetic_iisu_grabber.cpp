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
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */


#include "softkinetic_iisu_grabber.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/rgbd_image.h>

using namespace cv;

namespace ntk
{

void SoftKineticIisuGrabber :: onError(const SK::ErrorEvent& event)
{
    ntk_dbg(0) << "iisu error : " << event.getError().getDescription().ptr();
}

void SoftKineticIisuGrabber :: onDataFrame(const SK::DataFrameEvent& event)
{
    if (!m_running) // before the grabber starts.
        return;

    //we ask iisu for a new data frame
    SK::Result res = m_device->updateFrame();
    if (res.failed())
    {
        ntk_dbg(0) << "Failed updating frame";
        return;
    }

    if (m_depth_image.isValid())
    {
        SK::Image depth_image = m_depth_image.get();
        SK::ImageInfos depth_infos = depth_image.getImageInfos ();
        ntk_dbg_print (depth_infos.width, 2);
        ntk_dbg_print (depth_infos.height, 2);
        ntk_dbg_print (depth_infos.pixelTypeToString(), 2);
        ntk_dbg_print (depth_infos.imageTypeString(), 2);
        uint16_t* raw_values = reinterpret_cast<uint16_t*> (depth_image.getRAW());
        const uint16_t* last_raw_value = raw_values + depth_infos.width*depth_infos.height;
        float* depth_buffer = m_current_image.rawDepthRef().ptr<float>(0);
        while (raw_values != last_raw_value)
        {
            (*depth_buffer++) = (*raw_values++) / 1000.f;
        }

        m_depth_transmitted = false;
    }

    if (m_rgb_image.isValid())
    {
        SK::Image rgb_image = m_rgb_image.get();
        SK::ImageInfos rgb_infos = rgb_image.getImageInfos ();
        ntk_dbg_print (rgb_infos.bytesPerPixel(), 2);
        ntk_dbg_print (rgb_infos.channelsNb, 2);
        ntk_dbg_print (rgb_infos.pixels(), 2);
        ntk_dbg_print (rgb_infos.width, 2);
        ntk_dbg_print (rgb_infos.height, 2);
        ntk_dbg_print (rgb_infos.pixelTypeToString(), 2);
        ntk_dbg_print (rgb_infos.imageTypeString(), 2);
        ntk_dbg_print (m_confidence_filter_parameter.get(), 1);
        ntk_dbg_print (m_confidence_filter_min_threshold.get(), 1);
        ntk_dbg_print (m_edge_filter_parameter.get(), 1);
        ntk_dbg_print (m_smooth_filter_parameter.get(), 1);
        Vec4b* raw_values = reinterpret_cast<Vec4b*> (rgb_image.getRAW());
        const Vec4b* last_raw_value = raw_values + rgb_infos.width*rgb_infos.height;
        Vec3b* rgb_buffer = m_current_image.rawRgbRef().ptr<Vec3b>(0);
        while (raw_values != last_raw_value)
        {
            Vec4b rgba = (*raw_values++);
            (*rgb_buffer++) = Vec3b(rgba[0], rgba[1], rgba[2]);
        }
        m_rgb_transmitted = false;
    }

    // tell iisu we finished using data.
    m_device->releaseFrame();

    handleNewFrame ();
}

void SoftKineticIisuGrabber :: estimateCalibration()
{
    m_calib_data = new RGBDCalibration();

    int32_t rgb_width = m_rgb_width_parameter.get();
    int32_t rgb_height = m_rgb_height_parameter.get();

    int32_t depth_width = m_depth_width_parameter.get();
    int32_t depth_height = m_depth_height_parameter.get();

    double fx = depth_width / (2.0 * tan (m_depth_hfov.get() / 2.0));
    double fy = depth_height / (2.0 * tan (m_depth_vfov.get() / 2.0));

    double cx = depth_width / 2;
    double cy = depth_height / 2;

    m_calib_data->depth_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->depth_intrinsics);
    m_calib_data->depth_intrinsics(0,0) = fx;
    m_calib_data->depth_intrinsics(1,1) = fy;
    m_calib_data->depth_intrinsics(0,2) = cx;
    m_calib_data->depth_intrinsics(1,2) = cy;

    m_calib_data->depth_distortion = Mat1d(1,5);
    m_calib_data->depth_distortion = 0.;
#if 0
    m_calib_data->depth_distortion(0,0) = parameters.depthIntrinsics.k1;
    m_calib_data->depth_distortion(0,1) = parameters.depthIntrinsics.k2;
    m_calib_data->depth_distortion(0,2) = parameters.depthIntrinsics.p1;
    m_calib_data->depth_distortion(0,3) = parameters.depthIntrinsics.p2;
    m_calib_data->depth_distortion(0,4) = parameters.depthIntrinsics.k3;
    m_calib_data->zero_depth_distortion = false;
#endif

    m_calib_data->setRawRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->setRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->raw_depth_size = cv::Size(depth_width, depth_height);
    m_calib_data->depth_size = cv::Size(depth_width, depth_height);

#if 0
    float rgb_fx = parameters.colorIntrinsics.fx;
    float rgb_fy = parameters.colorIntrinsics.fy;
    float rgb_cx = parameters.colorIntrinsics.cx;
    float rgb_cy = parameters.colorIntrinsics.cy;
#endif
    double rgb_fx = rgb_width / (2.0 * tan (m_rgb_hfov.get() / 2.0));
    double rgb_fy = rgb_height / (2.0 * tan (m_rgb_vfov.get() / 2.0));

    double rgb_cx = rgb_width / 2;
    double rgb_cy = rgb_height / 2;

    m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->rgb_intrinsics);
    m_calib_data->rgb_intrinsics(0,0) = rgb_fx;
    m_calib_data->rgb_intrinsics(1,1) = rgb_fy;
    m_calib_data->rgb_intrinsics(0,2) = rgb_cx;
    m_calib_data->rgb_intrinsics(1,2) = rgb_cy;

    m_calib_data->rgb_distortion = Mat1d(1,5);
    m_calib_data->rgb_distortion = 0.;
#if 0
    m_calib_data->rgb_distortion(0,0) = parameters.colorIntrinsics.k1;
    m_calib_data->rgb_distortion(0,1) = parameters.colorIntrinsics.k2;
    m_calib_data->rgb_distortion(0,2) = parameters.colorIntrinsics.p1;
    m_calib_data->rgb_distortion(0,3) = parameters.colorIntrinsics.p2;
    m_calib_data->rgb_distortion(0,4) = parameters.colorIntrinsics.k3;
    m_calib_data->zero_rgb_distortion = false;
#endif

    m_calib_data->R = Mat1d(3,3);
    setIdentity(m_calib_data->R);

    // FIXME: these rotation parameters are not correctly interpreted.
    // They degrade the mapping..
#if 0
    m_calib_data->R(0,0) = parameters.extrinsics.r11;
    m_calib_data->R(0,1) = parameters.extrinsics.r12;
    m_calib_data->R(0,2) = parameters.extrinsics.r13;

    // Avoid the flipping applied on y. FIXME: discuss this with SoftKineticIisu.
    m_calib_data->R(1,0) = -parameters.extrinsics.r21;
    m_calib_data->R(1,1) = -parameters.extrinsics.r22;
    m_calib_data->R(1,2) = -parameters.extrinsics.r23;

    m_calib_data->R(2,0) = parameters.extrinsics.r31;
    m_calib_data->R(2,1) = parameters.extrinsics.r32;
    m_calib_data->R(2,2) = parameters.extrinsics.r33;
#endif

    m_calib_data->T = Mat1d(3,1);
    m_calib_data->T = 0.;

    // m_calib_data->T(0,0) = -parameters.extrinsics.t1;
    // m_calib_data->T(0,1) = -parameters.extrinsics.t2;
    // m_calib_data->T(0,2) = -parameters.extrinsics.t3;

    m_calib_data->depth_pose = new Pose3D();
    m_calib_data->depth_pose->setCameraParametersFromOpencv(m_calib_data->depth_intrinsics);

    m_calib_data->rgb_pose = new Pose3D();
    m_calib_data->rgb_pose->toRightCamera(m_calib_data->rgb_intrinsics,
                                          m_calib_data->R,
                                          m_calib_data->T);

    m_calib_data->updateDistortionMaps();
}

bool SoftKineticIisuGrabber :: registerData()
{
    m_depth_image = m_device->registerDataHandle<SK::Image>("SOURCE.CAMERA.DEPTH.Image");
    if (!m_depth_image.isValid())
    {
        ntk_dbg(0) << "Could not register depth data.";
        return false;
    }

    m_rgb_image = m_device->registerDataHandle<SK::Image>("SOURCE.CAMERA.COLOR.Image");
    if (!m_rgb_image.isValid())
    {
        ntk_dbg(0) << "Could not register color data.";
        return false;
    }

    return true;
}

bool SoftKineticIisuGrabber :: registerEvents()
{
    // a new dataframe has been computed by iisu
    SK::Result ret = m_iisuHandle->getEventManager().registerEventListener("DEVICE.DataFrame", *this, &SoftKineticIisuGrabber::onDataFrame);
    if (ret.failed())
    {
        ntk_dbg(0) << "Failed to register for data frame events!\n"
                   << "Error " << ret.getErrorCode() << ": " << ret.getDescription().ptr() << endl;
        return false;
    }

    // system errors events
    ret = m_iisuHandle->getEventManager().registerEventListener("SYSTEM.Error", *this, &SoftKineticIisuGrabber::onError);
    if (ret.failed())
    {
        ntk_dbg(0) << "Failed to register for system error events!\n"
                   << "Error " << ret.getErrorCode() << ": " << ret.getDescription().ptr() << endl;
        return false;
    }

    return true;
}

bool SoftKineticIisuGrabber :: registerParameters()
{
    m_cameraModel = m_device->registerParameterHandle<SK::String>("SOURCE.CAMERA.Model");
    m_depth_width_parameter = m_device->registerParameterHandle<int>("SOURCE.CAMERA.DEPTH.Width");
    m_depth_height_parameter = m_device->registerParameterHandle<int>("SOURCE.CAMERA.DEPTH.Height");
    m_rgb_width_parameter = m_device->registerParameterHandle<int>("SOURCE.CAMERA.COLOR.Width");
    m_rgb_height_parameter = m_device->registerParameterHandle<int>("SOURCE.CAMERA.COLOR.Height");

    m_rgb_hfov = m_device->registerParameterHandle<float>("SOURCE.CAMERA.COLOR.HFOV");
    m_rgb_vfov = m_device->registerParameterHandle<float>("SOURCE.CAMERA.COLOR.VFOV");
    m_depth_hfov = m_device->registerParameterHandle<float>("SOURCE.CAMERA.DEPTH.HFOV");
    m_depth_vfov = m_device->registerParameterHandle<float>("SOURCE.CAMERA.DEPTH.VFOV");

    m_confidence_filter_parameter = m_device->registerParameterHandle<bool>("SOURCE.FILTER.CONFIDENCE.Enabled");
    m_confidence_filter_min_threshold = m_device->registerParameterHandle<int>("SOURCE.FILTER.CONFIDENCE.MinThreshold");

    m_smooth_filter_parameter = m_device->registerParameterHandle<bool>("SOURCE.FILTER.SMOOTHING.Enabled");
    m_edge_filter_parameter = m_device->registerParameterHandle<bool>("SOURCE.FILTER.RECONSTRUCTION.Enabled");

    return true;
}

bool SoftKineticIisuGrabber :: connectToDevice()
{
    if (m_connected)
        return true;

    std::string dllLocation = getenv("IISU_SDK_DIR");
    dllLocation += "/bin";

    // get the working context
    SK::Context& context = SK::Context::Instance();

    // create an iisu configuration
    SK::IisuHandle::Configuration iisuConfiguration (dllLocation.c_str(), "iisu_config.xml");
    // you can customize the configuration here

    // create the handle according to the configuration structure
    SK::Return<SK::IisuHandle*> retHandle = context.createHandle(iisuConfiguration);
    if (retHandle.failed())
    {
        ntk_dbg (0) << "Failed to get iisu handle!\n"
                    << "Error " << retHandle.getErrorCode() << ": " << retHandle.getDescription().ptr();
        return false;
    }

    // get the iisu handle
    m_iisuHandle = retHandle.get();

    // create device configuration
    SK::Device::Configuration deviceConfiguration ;
    // you can customize the configuration here

    // create the device according to the configuration structure
    SK::Return<SK::Device*> retDevice = m_iisuHandle->initializeDevice(deviceConfiguration);
    if (retDevice.failed())
    {
        ntk_dbg (0) << "Failed to create device!\n"
                    << "Error " << retHandle.getErrorCode() << ": " << retHandle.getDescription().ptr();
        return false;
    }

    // get the device
    m_device = retDevice.get();

    bool ok = true;

    ok &= registerData();
    if (!ok)
    {
        ntk_dbg(0) << "register data failed.";
        return false;
    }

    ok &= registerEvents();
    if (!ok)
    {
        ntk_dbg(0) << "register events failed.";
        return false;
    }

    ok &= registerParameters();
    if (!ok)
    {
        ntk_dbg(0) << "register parameters failed.";
        return false;
    }

    // print camera infos
    ntk_dbg(1) << "Camera Model : " << m_cameraModel.get().ptr();
    ntk_dbg(1) << "Depth width : " << m_depth_width_parameter.get();
    ntk_dbg(1) << "Depth height : " << m_depth_height_parameter.get();

    m_confidence_filter_min_threshold.set(200);

    estimateCalibration ();
    m_calib_data->saveToFile("debug_calib.yml");

    SK::Result devStart = m_device->start();
    if (devStart.failed())
    {
        ntk_dbg (0) << "Failed to start device!\n"
                    << "Error " << devStart.getErrorCode() << ": " << devStart.getDescription().ptr();
        return false;
    }

    // FIXME: how can I get the serial number?
    // m_camera_serial = m_depth_node.getSerialNumber();
    m_camera_serial = "softkinetic_0";

    m_connected = true;
    return true;
}

bool SoftKineticIisuGrabber :: disconnectFromDevice()
{
    if (!m_connected)
        return true;

    if (m_device)
    {
        //stop iisu processing
        SK::Result devStop = m_device->stop();
        if (devStop.failed())
        {
            ntk_dbg(0) << "Failed to stop device!\n"
                       << "Error " << devStop.getErrorCode() << ": " << devStop.getDescription().ptr();
        }
        // clear the device pointer
        m_device = 0;
    }

    // if we have iisu handle
    if (m_iisuHandle)
    {

        // destroy the iisu handle
        SK::Result res = SK::Context::Instance().destroyHandle(*m_iisuHandle);
        if (res.failed())
        {
            ntk_dbg(0) << "Failed to destroy handle!\n"
                       << "Error " << res.getErrorCode() << ": " << res.getDescription().ptr();
        }
        // cleat the iisu handle
        m_iisuHandle = 0;
    }

    // finalize context
    SK::Context::Instance().finalize();

    m_connected = false;
    return true;
}

void SoftKineticIisuGrabber :: handleNewFrame()
{
    if (m_depth_transmitted || m_rgb_transmitted)
    {
        ntk_dbg (1) << "NOT dirty, doing nothing\n";
        return;
    }

    // std::clog << "Both channels are dirty, creating a new RGBDImage\n";

    {
        int64 grab_time = ntk::Time::getMillisecondCounter();
        ntk_dbg_print(grab_time - m_last_grab_time, 2);
        m_last_grab_time = grab_time;
        QWriteLocker locker(&m_lock);
        // FIXME: ugly hack to handle the possible time
        // gaps between rgb and IR frames in dual mode.
        m_current_image.setTimestamp(getCurrentTimestamp());
        m_current_image.swap(m_rgbd_image);
        m_rgb_transmitted = true;
        m_depth_transmitted = true;
    }



    advertiseNewFrame();
}

void SoftKineticIisuGrabber :: run()
{
    setThreadShouldExit(false);
    m_current_image.setCalibration(m_calib_data);
    m_rgbd_image.setCalibration(m_calib_data);

    m_current_image.setCameraSerial(m_camera_serial);
    m_rgbd_image.setCameraSerial(m_camera_serial);

    m_rgbd_image.setGrabberType(grabberType());
    m_current_image.setGrabberType(grabberType());

    int32_t color_width = m_rgb_width_parameter.get();
    int32_t color_height = m_rgb_height_parameter.get();

    int depth_width = m_depth_width_parameter.get();
    int depth_height = m_depth_height_parameter.get();

    m_rgbd_image.rawRgbRef() = Mat3b(color_height, color_width);
    m_rgbd_image.rawDepthRef() = Mat1f(depth_height, depth_width);

    m_current_image.rawRgbRef() = Mat3b(color_height, color_width);
    m_current_image.rawDepthRef() = Mat1f(depth_height, depth_width);

    m_current_image.setCalibration(m_calib_data);
    m_rgbd_image.setCalibration(m_calib_data);

    m_running = true;

    while (!threadShouldExit())
    {
        ntk::sleep (100);
    }

    m_running = false;
}

} // ntk
