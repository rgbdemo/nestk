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


#include "softkinetic_grabber.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/time.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/camera/rgbd_image.h>

using namespace cv;
using namespace DepthSense;

#define SOFTKINETIC_CONFIDENCE_AS_COLOR 1

namespace ntk
{

static void onDeviceConnected_cb(Context context, Context::DeviceAddedData data, SoftKineticGrabber* that)
{
    that->onDeviceConnected(data);
}

static void onDeviceDisconnected_cb(Context context, Context::DeviceRemovedData data, SoftKineticGrabber* that)
{
    that->onDeviceDisconnected(data);
}

static void onNodeConnected_cb(Device device, Device::NodeAddedData data, SoftKineticGrabber* that)
{
    that->onNodeConnected(data);
}

static void onNodeDisconnected_cb(Device device, Device::NodeRemovedData data, SoftKineticGrabber* that)
{
    that->onNodeDisconnected(data);
}

static void onNewColorSample_cb(ColorNode node, ColorNode::NewSampleReceivedData data, SoftKineticGrabber* that)
{
    that->onNewColorSample(data);
}

static void onNewDepthSample_cb(DepthNode node, DepthNode::NewSampleReceivedData data, SoftKineticGrabber* that)
{
    that->onNewDepthSample(data);
}

/*****************************************************************************/

inline cv::Vec3b yuv_to_bgr888(int y, int u, int v)
{
    cv::Vec3b r;
    int C = y - 16;
    int D = u - 128;
    int E = v - 128;
    r[2] = ntk::saturate_to_range((298*C + 409*E + 128) >> 8, 0, 255);
    r[1] = ntk::saturate_to_range((298*C - 100*D - 208*E + 128) >> 8, 0, 255);
    r[0] = ntk::saturate_to_range((298*C + 516*D + 128) >> 8, 0, 255);
    return r;
}

/*****************************************************************************/

void SoftKineticGrabber :: estimateCalibration(DepthSense::StereoCameraParameters& parameters)
{
    m_calib_data = new RGBDCalibration();

    double fx = parameters.depthIntrinsics.fx;
    double fy = parameters.depthIntrinsics.fy;

    double cx = parameters.depthIntrinsics.cx;
    double cy = parameters.depthIntrinsics.cy;

    ntk_assert(m_color_node.getConfiguration().frameFormat == FRAME_FORMAT_VGA, "Unknown color image format.");
    int32_t rgb_width = 640;
    int32_t rgb_height = 480;
    FrameFormat_toResolution (m_color_node.getConfiguration().frameFormat, &rgb_width, &rgb_height);

    int32_t depth_width = -1;
    int32_t depth_height = -1;
    FrameFormat_toResolution (m_depth_node.getConfiguration().frameFormat, &depth_width, &depth_height);

#ifdef SOFTKINETIC_CONFIDENCE_AS_COLOR
    rgb_width = depth_width;
    rgb_height = depth_height;
#endif

    m_calib_data->depth_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->depth_intrinsics);
    m_calib_data->depth_intrinsics(0,0) = fx;
    m_calib_data->depth_intrinsics(1,1) = fy;
    m_calib_data->depth_intrinsics(0,2) = cx;
    m_calib_data->depth_intrinsics(1,2) = cy;

    m_calib_data->depth_distortion = Mat1d(1,5);
    m_calib_data->depth_distortion(0,0) = parameters.depthIntrinsics.k1;
    m_calib_data->depth_distortion(0,1) = parameters.depthIntrinsics.k2;
    m_calib_data->depth_distortion(0,2) = parameters.depthIntrinsics.p1;
    m_calib_data->depth_distortion(0,3) = parameters.depthIntrinsics.p2;
    m_calib_data->depth_distortion(0,4) = parameters.depthIntrinsics.k3;
    m_calib_data->zero_depth_distortion = false;

    m_calib_data->setRawRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->setRgbSize(cv::Size(rgb_width, rgb_height));
    m_calib_data->raw_depth_size = cv::Size(depth_width, depth_height);
    m_calib_data->depth_size = cv::Size(depth_width, depth_height);

#ifndef SOFTKINETIC_CONFIDENCE_AS_COLOR
    float rgb_fx = parameters.colorIntrinsics.fx;
    float rgb_fy = parameters.colorIntrinsics.fy;
    float rgb_cx = parameters.colorIntrinsics.cx;
    float rgb_cy = parameters.colorIntrinsics.cy;
#else
    float rgb_fx = parameters.depthIntrinsics.fx;
    float rgb_fy = parameters.depthIntrinsics.fy;
    float rgb_cx = parameters.depthIntrinsics.cx;
    float rgb_cy = parameters.depthIntrinsics.cy;
#endif

    m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
    setIdentity(m_calib_data->rgb_intrinsics);
    m_calib_data->rgb_intrinsics(0,0) = rgb_fx;
    m_calib_data->rgb_intrinsics(1,1) = rgb_fy;
    m_calib_data->rgb_intrinsics(0,2) = rgb_cx;
    m_calib_data->rgb_intrinsics(1,2) = rgb_cy;

    m_calib_data->rgb_distortion = Mat1d(1,5);
#ifndef SOFTKINETIC_CONFIDENCE_AS_COLOR
    m_calib_data->rgb_distortion(0, 0) = parameters.colorIntrinsics.k1;
    m_calib_data->rgb_distortion(0, 1) = parameters.colorIntrinsics.k2;
    m_calib_data->rgb_distortion(0, 2) = parameters.colorIntrinsics.p1;
    m_calib_data->rgb_distortion(0, 3) = parameters.colorIntrinsics.p2;
    m_calib_data->rgb_distortion(0, 4) = parameters.colorIntrinsics.k3;
#else
    m_calib_data->rgb_distortion(0, 0) = parameters.depthIntrinsics.k1;
    m_calib_data->rgb_distortion(0, 1) = parameters.depthIntrinsics.k2;
    m_calib_data->rgb_distortion(0, 2) = parameters.depthIntrinsics.p1;
    m_calib_data->rgb_distortion(0, 3) = parameters.depthIntrinsics.p2;
    m_calib_data->rgb_distortion(0, 4) = parameters.depthIntrinsics.k3;
#endif
    m_calib_data->zero_rgb_distortion = false;

    m_calib_data->R = Mat1d(3,3);
    setIdentity(m_calib_data->R);

    // FIXME: these rotation parameters are not correctly interpreted.
    // They degrade the mapping..
#if 0
    m_calib_data->R(0,0) = parameters.extrinsics.r11;
    m_calib_data->R(0,1) = parameters.extrinsics.r12;
    m_calib_data->R(0,2) = parameters.extrinsics.r13;

    // Avoid the flipping applied on y. FIXME: discuss this with softkinetic.
    m_calib_data->R(1,0) = -parameters.extrinsics.r21;
    m_calib_data->R(1,1) = -parameters.extrinsics.r22;
    m_calib_data->R(1,2) = -parameters.extrinsics.r23;

    m_calib_data->R(2,0) = parameters.extrinsics.r31;
    m_calib_data->R(2,1) = parameters.extrinsics.r32;
    m_calib_data->R(2,2) = parameters.extrinsics.r33;
#endif

    m_calib_data->T = Mat1d(3,1);
    m_calib_data->T = 0.;

#ifndef SOFTKINETIC_CONFIDENCE_AS_COLOR
    m_calib_data->T(0,0) = -parameters.extrinsics.t1;
    m_calib_data->T(1,0) = -parameters.extrinsics.t2;
    m_calib_data->T(2,0) = -parameters.extrinsics.t3;
#endif

    m_calib_data->depth_pose = new Pose3D();
    m_calib_data->depth_pose->setCameraParametersFromOpencv(m_calib_data->depth_intrinsics);

    m_calib_data->rgb_pose = new Pose3D();
    m_calib_data->rgb_pose->toRightCamera(m_calib_data->rgb_intrinsics,
                                          m_calib_data->R,
                                          m_calib_data->T);

    m_calib_data->updateDistortionMaps();
}

void SoftKineticGrabber::onNewColorSample(ColorNode::NewSampleReceivedData data)
{
    if (!m_connected) // not yet in running mode.
        return;

#ifdef SOFTKINETIC_CONFIDENCE_AS_COLOR
    return;
#endif

    // std::clog << cv::format("Color: %d pixels\n", data.colorMap.size());
    cv::Vec3b* rgb_buffer = m_current_image.rawRgbRef().ptr<cv::Vec3b>(0);
    for (int i = 0; i < data.colorMap.size(); i += 4)
    {
        int y1 = data.colorMap[i];
        int u = data.colorMap[i+1];
        int y2 = data.colorMap[i+2];
        int v = data.colorMap[i+3];
        *rgb_buffer++ = yuv_to_bgr888(y1, u, v);
        *rgb_buffer++ = yuv_to_bgr888(y2, u, v);
    }
    m_rgb_transmitted = false;
    handleNewFrame();
}

void SoftKineticGrabber::onNewDepthSample(DepthNode::NewSampleReceivedData data)
{
    if (!m_connected) // not yet in running mode.
    {
        std::clog << "Depth sample received, calibrating.";
        if (!m_calib_data)
            estimateCalibration(data.stereoCameraParameters);

        m_calib_data->setMinDepthInMeters (0.15f);
        m_calib_data->setMaxDepthInMeters (2.0f);

        // Stop sending events until the connection happens.
        m_context.quit();
        return;
    }

#ifdef SOFTKINETIC_CONFIDENCE_AS_COLOR
    const int16_t* confidence_map_values = data.confidenceMap;
    const int16_t* last_confidence_value = data.confidenceMap + data.confidenceMap.size();
    float* confidence_buffer = m_current_image.rawAmplitudeRef().ptr<float>();
    while (confidence_map_values != last_confidence_value)
    {
        (*confidence_buffer++) = *confidence_map_values++;
    }
    m_current_image.rawRgbRef() = ntk::toMat3b(ntk::normalize_toMat1b(m_current_image.rawAmplitude()));
    m_rgb_transmitted = false;
#endif

    // std::clog << cv::format("Depth: %d\n", data.depthMap.size());
    float* depth_buffer = m_current_image.rawDepthRef().ptr<float>(0);
    const int16_t* raw_values = data.depthMap;
    const int16_t* last_raw_value = data.depthMap + data.depthMap.size();
    while (raw_values != last_raw_value)
    {
        int16_t saturated_value = *raw_values++;
        if (saturated_value > 31999)
            saturated_value = 0;
        (*depth_buffer++) = saturated_value / 1000.f;
    }
    m_depth_transmitted = false;
    handleNewFrame();
#if 0
    // Project some 3D points in the Color Frame
    if (!g_pProjHelper)
    {
        g_pProjHelper = new ProjectionHelper (data.stereoCameraParameters);
        g_scp = data.stereoCameraParameters;
    }
    else if (g_scp != data.stereoCameraParameters)
    {
        g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
        g_scp = data.stereoCameraParameters;
    }

    int32_t w, h;
    FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);
    int cx = w/2;
    int cy = h/2;

    Vertex p3DPoints[4];

    p3DPoints[0] = data.vertices[(cy-h/4)*w+cx-w/4];
    p3DPoints[1] = data.vertices[(cy-h/4)*w+cx+w/4];
    p3DPoints[2] = data.vertices[(cy+h/4)*w+cx+w/4];
    p3DPoints[3] = data.vertices[(cy+h/4)*w+cx-w/4];

    Point2D p2DPoints[4];
    g_pProjHelper->get2DCoordinates ( p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);
#endif
}

void SoftKineticGrabber :: onDeviceConnected(Context::DeviceAddedData data)
{
    std::clog << "Device connected";
#if 0
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
#endif
}

void SoftKineticGrabber :: onDeviceDisconnected(Context::DeviceRemovedData data)
{
    // g_bDeviceFound = false;
    std::clog << "Device disconnected";
}

void SoftKineticGrabber :: configureNode(Node node)
{
    if ((node.is<DepthNode>()) && (!m_depth_node.isSet()))
    {
        m_depth_node = node.as<DepthNode>();
        m_depth_node.newSampleReceivedEvent().connect(&onNewDepthSample_cb, this);
        DepthNode::Configuration config = m_depth_node.getConfiguration();
        config.frameFormat = FRAME_FORMAT_QQVGA;
        config.framerate = 30;
        config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
        // config.mode = DepthNode::CAMERA_MODE_LONG_RANGE;
        config.saturation = true;
        m_depth_node.setEnableDepthMap(true);
#ifdef SOFTKINETIC_CONFIDENCE_AS_COLOR
        m_depth_node.setEnableConfidenceMap(true);
#endif
        try
        {
            m_context.requestControl(m_depth_node,0);
            m_depth_node.setConfidenceThreshold(150);
            m_depth_node.setConfiguration(config);
        }
        catch (const std::exception& e)
        {
            std::clog << cv::format("Exception when configuring depth node: %s\n", e.what());
        }
        m_context.registerNode(node);
    }
    else if ((node.is<ColorNode>())&&(!m_color_node.isSet()))
    {
        m_color_node = node.as<ColorNode>();
        m_color_node.newSampleReceivedEvent().connect(&onNewColorSample_cb, this);

        ColorNode::Configuration config = m_color_node.getConfiguration();
        config.frameFormat = FRAME_FORMAT_VGA;
        // config.compression = COMPRESSION_TYPE_MJPEG;
        // config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
        config.framerate = 30;

        m_color_node.setEnableColorMap(true);
        try
        {
            m_context.requestControl(m_color_node,0);
            m_color_node.setConfiguration(config);
        }
        catch (const std::exception& e)
        {
            std::clog << cv::format("Color node configuration exception: %s\n", e.what());
        }
        m_context.registerNode(node);
    }
}

void SoftKineticGrabber :: onNodeConnected(Device::NodeAddedData data)
{
    configureNode(data.node);
}

void SoftKineticGrabber :: onNodeDisconnected(Device::NodeRemovedData data)
{
    printf("Node disconnected\n");
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == m_color_node))
        m_color_node.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == m_depth_node))
        m_depth_node.unset();
}

bool SoftKineticGrabber :: connectToDevice()
{
    if (m_connected)
        return true;

    m_context = Context::create("localhost");
    m_context.deviceAddedEvent().connect(&onDeviceConnected_cb, this);
    m_context.deviceRemovedEvent().connect(&onDeviceDisconnected_cb, this);

    // Get the list of currently connected devices
    std::vector<Device> devices = m_context.getDevices();
    ntk_dbg_print(devices.size(), 1);

    // We are only interested in the first device
    if (devices.size() <= m_device_id)
        return false;

    {
        devices[m_device_id].nodeAddedEvent().connect(&onNodeConnected_cb, this);
        devices[m_device_id].nodeRemovedEvent().connect(&onNodeDisconnected_cb, this);

        std::vector<Node> nodes = devices[m_device_id].getNodes();

        std::clog << cv::format("Found %u nodes\n",nodes.size());

        for (int n = 0; n < (int)nodes.size();n++)
            configureNode(nodes[n]);
    }

    if (!m_color_node.isSet())
    {
        std::clog << "Color node could not be set.";
        return false;
    }

    if (!m_depth_node.isSet())
    {
        std::clog << "Depth node could not be set.";
        return false;
    }

    m_context.startNodes();
    m_context.run();

    m_camera_serial = m_depth_node.getSerialNumber();

    m_connected = true;
    return true;
}

bool SoftKineticGrabber :: disconnectFromDevice()
{
    if (!m_connected)
        return true;

    // Exit requested.
    m_context.quit();
    m_context.stopNodes();
    if (m_color_node.isSet()) m_context.unregisterNode(m_color_node);
    if (m_depth_node.isSet()) m_context.unregisterNode(m_depth_node);
    m_connected = false;
    return true;
}

bool SoftKineticGrabber::hasDll()
{
#ifdef _MSC_VER
    // Trigger SoftKinetic DLL loading by calling one of its functions.
    __try
    {
        int32_t rgb_width, rgb_height;
        FrameFormat_toResolution (FRAME_FORMAT_VGA, &rgb_width, &rgb_height);
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

void SoftKineticGrabber :: handleNewFrame()
{
    if (m_depth_transmitted || m_rgb_transmitted)
    {
        // std::clog << "NOT dirty, doing nothing\n";
        return;
    }

    // std::clog << "Both channels are dirty, creating a new RGBDImage\n";

    if (threadShouldExit())
    {
        disconnectFromDevice();
    }

    {
        QWriteLocker locker(&m_lock);
        m_current_image.setTimestamp(getCurrentTimestamp());
        // FIXME: hack to handle the possible time
        // gaps between rgb and IR frames in dual mode.
        m_current_image.swap(m_rgbd_image);
        m_rgb_transmitted = true;
        m_depth_transmitted = true;
    }

    advertiseNewFrame();
}

void SoftKineticGrabber :: run()
{
    setThreadShouldExit(false);
    m_current_image.setCalibration(m_calib_data);
    m_rgbd_image.setCalibration(m_calib_data);

    m_current_image.setCameraSerial(m_camera_serial);
    m_rgbd_image.setCameraSerial(m_camera_serial);

    m_rgbd_image.setGrabberType(grabberType());
    m_current_image.setGrabberType(grabberType());

    ntk_assert(m_color_node.getConfiguration().frameFormat == FRAME_FORMAT_VGA, "Unknown color image format.");
    int32_t color_width = 640;
    int32_t color_height = 480;
    FrameFormat_toResolution (m_color_node.getConfiguration().frameFormat, &color_width, &color_height);

    int depth_width = -1;
    int depth_height = -1;
    FrameFormat_toResolution (m_depth_node.getConfiguration().frameFormat, &depth_width, &depth_height);

#ifdef SOFTKINETIC_CONFIDENCE_AS_COLOR
    color_width = depth_width;
    color_height = depth_height;
#endif

    m_rgbd_image.rawRgbRef() = Mat3b(color_height, color_width);
    m_rgbd_image.rawDepthRef() = Mat1f(depth_height, depth_width);

    m_current_image.rawRgbRef() = Mat3b(color_height, color_width);
    m_current_image.rawDepthRef() = Mat1f(depth_height, depth_width);

#ifdef SOFTKINETIC_CONFIDENCE_AS_COLOR
    m_rgbd_image.rawAmplitudeRef() = Mat1f(depth_height, depth_width);
    m_current_image.rawAmplitudeRef() = Mat1f(depth_height, depth_width);
#endif

    m_current_image.setCalibration(m_calib_data);
    m_rgbd_image.setCalibration(m_calib_data);

    m_context.run();
}

} // ntk
