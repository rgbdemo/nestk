/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "openni_grabber.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/gesture/body_event.h>
#include <ntk/geometry/pose_3d.h>

#include <XnVCircleDetector.h>
#include <XnLog.h>

#include <QTemporaryFile>
#include <string>

using namespace cv;
using namespace ntk;

#include "openni_grabber_internals.hxx"


QMutex ntk::OpenniGrabber::m_ni_mutex;

namespace
{

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
    ntk::OpenniGrabber* grabber = reinterpret_cast<ntk::OpenniGrabber*>(pCookie);
    grabber->calibrationStartedCallback(nId);
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    ntk::OpenniGrabber* grabber = reinterpret_cast<ntk::OpenniGrabber*>(pCookie);
    grabber->newUserCallback(nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    ntk::OpenniGrabber* grabber = reinterpret_cast<ntk::OpenniGrabber*>(pCookie);
    grabber->lostUserCallback(nId);
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
    ntk::OpenniGrabber* grabber = reinterpret_cast<ntk::OpenniGrabber*>(pCookie);
    grabber->userPoseDetectedCallback(nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
    ntk::OpenniGrabber* grabber = reinterpret_cast<ntk::OpenniGrabber*>(pCookie);
    grabber->calibrationFinishedCallback(nId, bSuccess);
}

} // namespace anonymous

// Used in openni XnModuleLoader.cpp.
#ifdef NESTK_USE_CUSTOM_OPENNI
    // Will be defined in OpenNI.
    extern const char* xn_modules_file;
#else
    const char* xn_modules_file = "config/modules.xml";
#endif

namespace ntk
{

OpenniGrabber :: OpenniGrabber(OpenniDriver& driver, int camera_id) :
    m_driver(driver),
    m_subsampling_factor(1),
    m_camera_id(camera_id),
    m_need_pose_to_calibrate(false),
    m_max_num_users(15),
    m_body_event_detector(0),
    m_high_resolution(false),
    m_mirrored(false),
    m_custom_bayer_decoding(true),
    m_xml_config_file(DEFAULT_XML_CONFIG_FILE),
    m_track_users(true),
    m_get_infrared(false)
{
}

void OpenniGrabber :: setIRMode(bool ir)
{
    QMutexLocker _(&m_ni_mutex);

    if (ir == m_get_infrared)
        return;

    XnMapOutputMode dmomVGA;
    dmomVGA.nFPS = 30;
    dmomVGA.nXRes = 640;
    dmomVGA.nYRes = 480;

    XnMapOutputMode dmomQVGA;
    dmomQVGA.nFPS = 30;
    dmomQVGA.nXRes = 320;
    dmomQVGA.nYRes = 240;

    m_get_infrared = ir;
    if (m_get_infrared)
    {
        m_ni_rgb_generator.StopGenerating();
        // m_ni_depth_generator.GetAlternativeViewPointCap().ResetViewPoint();
        // m_ni_depth_generator.SetMapOutputMode(dmomQVGA);
        m_ni_ir_generator.StartGenerating();
    }
    else
    {
        m_ni_ir_generator.StopGenerating();
        // m_ni_depth_generator.GetAlternativeViewPointCap().SetViewPoint(m_ni_rgb_generator);
        m_ni_rgb_generator.StartGenerating();
    }
}

void OpenniGrabber :: setSubsamplingFactor(int factor)
{
    if (640 % factor > 0 || 480 % factor > 0)
    {
        ntk_throw_exception("Invalid subsampling factor. Must give a round subsampled size.");
    }
    m_subsampling_factor = factor;
}

const std::string OpenniGrabber :: DEFAULT_XML_CONFIG_FILE =  "config/NestkConfig.xml";

void OpenniGrabber :: set_xml_config_file(const std::string & xml_filename)
{
    m_xml_config_file = xml_filename;
}

bool OpenniGrabber :: connectToDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);
    ntk_dbg(1) << format("[Kinect %x] connecting", this);

    XnStatus status;
    xn::NodeInfoList device_node_info_list;
    status = m_driver.niContext().EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, device_node_info_list);
    m_driver.checkXnError(status, "Cannot enumerate devices.");
    xn::NodeInfoList::Iterator nodeIt = device_node_info_list.Begin();
    int current_id = 0;
    for (; current_id < m_camera_id && nodeIt != device_node_info_list.End (); ++nodeIt, ++current_id) {}
    ntk_throw_exception_if(current_id != m_camera_id, "Cannot find device.");
    xn::NodeInfo deviceInfo = *nodeIt;
    status = m_driver.niContext().CreateProductionTree(deviceInfo, m_ni_device);
    m_driver.checkXnError(status, "Create Device Node");
    const XnProductionNodeDescription& description = deviceInfo.GetDescription();
    ntk_dbg(1) << format("device: vendor %s name %s, instance %s",
                         description.strVendor, description.strName, deviceInfo.GetInstanceName());

    xn::Query query;
    query.AddNeededNode(deviceInfo.GetInstanceName());

    status = m_ni_depth_generator.Create(m_driver.niContext(), &query);
    m_driver.checkXnError(status, "Create depth generator");
    XnMapOutputMode depth_mode;
    depth_mode.nXRes = 640;
    depth_mode.nYRes = 480;
    depth_mode.nFPS = 30;
    m_ni_depth_generator.SetMapOutputMode(depth_mode);

    status = m_ni_rgb_generator.Create(m_driver.niContext(), &query);
    m_driver.checkXnError(status, "Create image generator");

    status = m_ni_rgb_generator.SetIntProperty ("Resolution", 1);
    m_driver.checkXnError(status, "Resolution");

    status = m_driver.niContext().SetGlobalMirror(m_mirrored);
    m_driver.checkXnError(status, "Mirror");

    XnMapOutputMode rgb_mode;
    if (m_high_resolution)
    {
        rgb_mode.nFPS = 15;
        rgb_mode.nXRes = 1280;
        rgb_mode.nYRes = 1024;
    }
    else
    {
        rgb_mode.nXRes = 640;
        rgb_mode.nYRes = 480;
        rgb_mode.nFPS = 30;
    }
    m_ni_rgb_generator.SetMapOutputMode(rgb_mode);

    ntk_throw_exception_if(!m_ni_depth_generator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT), "Cannot register images.");
    m_ni_depth_generator.GetAlternativeViewPointCap().SetViewPoint(m_ni_rgb_generator);

    status = m_ni_ir_generator.Create(m_driver.niContext(), &query);
    m_driver.checkXnError(status, "Create infrared generator");
    XnMapOutputMode ir_mode;
    ir_mode.nFPS = 15;
    ir_mode.nXRes = 1280;
    ir_mode.nYRes = 1024;
    m_ni_ir_generator.SetMapOutputMode(ir_mode);

    if (m_track_users)
    {
        status = m_ni_user_generator.Create(m_driver.niContext(), &query);
        m_driver.checkXnError(status, "Create user generator");
        status = m_ni_hands_generator.Create(m_driver.niContext(), &query);
        m_driver.checkXnError(status, "Create hands generator");
        status = m_ni_gesture_generator.Create(m_driver.niContext(), &query);
        m_driver.checkXnError(status, "Create gestures generator");
    }

    if (m_track_users)
    {
        ntk_throw_exception_if(!m_ni_user_generator.IsCapabilitySupported(XN_CAPABILITY_SKELETON),
                               "Supplied user generator doesn't support skeleton.");

        XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
        m_ni_user_generator.RegisterUserCallbacks(User_NewUser, User_LostUser, this, hUserCallbacks);
        m_ni_user_generator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, this, hCalibrationCallbacks);

        if (m_ni_user_generator.GetSkeletonCap().NeedPoseForCalibration())
        {
            m_need_pose_to_calibrate = true;
            if (!m_ni_user_generator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
                ntk_throw_exception("Pose required, but not supported\n");
            m_ni_user_generator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, this, hPoseCallbacks);
            m_ni_user_generator.GetSkeletonCap().GetCalibrationPose(m_calibration_pose);
        }

        m_ni_user_generator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

        if (m_body_event_detector)
            m_body_event_detector->initialize(m_driver.niContext(), m_ni_depth_generator);
    }

    status = m_ni_depth_generator.StartGenerating();
    m_driver.checkXnError(status, "Depth::StartGenerating");

    status = m_ni_rgb_generator.StartGenerating();
    m_driver.checkXnError(status, "RGB::StartGenerating");

    if (m_track_users)
    {
        status = m_ni_user_generator.StartGenerating();
        m_driver.checkXnError(status, "User::StartGenerating");

        status = m_ni_hands_generator.StartGenerating();
        m_driver.checkXnError(status, "Hands::StartGenerating");

        status = m_ni_gesture_generator.StartGenerating();
        m_driver.checkXnError(status, "Gesture::StartGenerating");
    }

    if (m_custom_bayer_decoding)
    {
        // Grayscale to get raw Bayer pattern.
        status = m_ni_rgb_generator.SetIntProperty ("InputFormat", 6);
        m_driver.checkXnError(status, "Change input format");

        status = m_ni_rgb_generator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
        m_driver.checkXnError(status, "Change pixel format");
    }

    waitAndUpdateActiveGenerators();
    if (!m_calib_data)
        estimateCalibration();

    m_connected = true;
    return true;
}

bool OpenniGrabber :: disconnectFromDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);
    ntk_dbg(1) << format("[Kinect %x] disconnecting", this);

    m_ni_depth_generator.StopGenerating();
    m_ni_rgb_generator.StopGenerating();
    m_ni_ir_generator.StopGenerating();
    if (m_track_users)
    {
        m_ni_user_generator.StopGenerating();
        m_ni_hands_generator.StopGenerating();
        m_ni_gesture_generator.StopGenerating();
    }
    if (m_body_event_detector)
        m_body_event_detector->shutDown();

    m_ni_depth_generator.Release();
    m_ni_rgb_generator.Release();
    m_ni_ir_generator.Release();
    m_ni_user_generator.Release();
    m_ni_hands_generator.Release();
    m_ni_gesture_generator.Release();
    m_ni_device.Release();
    m_connected = false;
    return true;
}

void OpenniGrabber :: waitAndUpdateActiveGenerators()
{
    // If there is only one device, call this global function.
    if (m_driver.numDevices() == 1)
    {
		m_driver.niContext().WaitOneUpdateAll(m_ni_depth_generator);
        return;
    }

    // Multiple kinect, only wait for our stream.
    m_ni_depth_generator.WaitAndUpdateData();
    if (m_get_infrared)
        m_ni_ir_generator.WaitAndUpdateData();
    else
        m_ni_rgb_generator.WaitAndUpdateData();

    // FIXME: for some reason, hand events are not generated using this.
    // Only WaitAndUpdateAll generates the events.
    if (m_track_users)
    {
        m_ni_user_generator.WaitAndUpdateData();
        m_ni_hands_generator.WaitAndUpdateData();
        m_ni_gesture_generator.WaitAndUpdateData();
    }
}

void OpenniGrabber :: estimateCalibration()
{
    XnPoint3D p;
    p.X = 0; p.Y = 0; p.Z = -1;
    m_ni_depth_generator.ConvertProjectiveToRealWorld(1, &p, &p);

    p.X = 0; p.Y = 0; p.Z = -1;
    m_ni_depth_generator.ConvertRealWorldToProjective(1, &p, &p);
    double cx = p.X;
    double cy = p.Y;

    p.X = 1; p.Y = 1; p.Z = -1;
    m_ni_depth_generator.ConvertRealWorldToProjective(1, &p, &p);

    double fx = -(p.X-cx);
    double fy = p.Y-cy;

    // These factors were estimated using chessboard calibration.
    // They seem to accurately correct the bias in object sizes output by
    // the default parameters.
    const double f_correction_factor = 528.0/575.8;
    fx *= f_correction_factor;
    fy *= f_correction_factor;

    const double cy_correction_factor = 267.0/240.0;
    cy *= cy_correction_factor;

    fx /= m_subsampling_factor;
    fy /= m_subsampling_factor;
    cx /= m_subsampling_factor;
    cy /= m_subsampling_factor;

    m_calib_data = new RGBDCalibration();

    xn::DepthMetaData depthMD;
    m_ni_depth_generator.GetMetaData(depthMD);

    xn::ImageMetaData rgbMD;
    m_ni_rgb_generator.GetMetaData(rgbMD);

    int depth_width = depthMD.XRes() / m_subsampling_factor;
    int depth_height = depthMD.YRes() / m_subsampling_factor;

    int rgb_width = rgbMD.XRes();
    int rgb_height = rgbMD.YRes();

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

    m_calib_data->camera_type = "kinect-ni";
}

void OpenniGrabber :: run()
{
    m_should_exit = false;
    m_current_image.setCalibration(m_calib_data);
    m_rgbd_image.setCalibration(m_calib_data);

    m_rgbd_image.rawRgbRef() = Mat3b(m_calib_data->rawRgbSize());
    m_rgbd_image.rawDepthRef() = Mat1f(m_calib_data->raw_depth_size);
    m_rgbd_image.rawIntensityRef() = Mat1f(m_calib_data->rawRgbSize());

    m_rgbd_image.rawIntensityRef() = 0.f;
    m_rgbd_image.rawDepthRef() = 0.f;
    m_rgbd_image.rawRgbRef() = Vec3b(0,0,0);

    m_rgbd_image.rgbRef() = m_rgbd_image.rawRgbRef();
    m_rgbd_image.depthRef() = m_rgbd_image.rawDepthRef();
    m_rgbd_image.intensityRef() = m_rgbd_image.rawIntensityRef();

    m_rgbd_image.userLabelsRef() = cv::Mat1b(m_calib_data->raw_depth_size);
    m_rgbd_image.userLabelsRef() = 0u;

    if (m_track_users)
        m_rgbd_image.setSkeletonData(new Skeleton());

    m_current_image.rawRgbRef() = Mat3b(m_calib_data->rawRgbSize());
    m_current_image.rawRgbRef() = Vec3b(0,0,0);
    m_current_image.rawDepthRef() = Mat1f(m_calib_data->raw_depth_size);
    m_current_image.rawDepthRef() = 0.f;
    m_current_image.rawIntensityRef() = Mat1f(m_calib_data->rawRgbSize());
    m_current_image.rawIntensityRef() = 0.f;

    m_current_image.rgbRef() = m_current_image.rawRgbRef();
    m_current_image.depthRef() = m_current_image.rawDepthRef();
    m_current_image.intensityRef() = m_current_image.rawIntensityRef();

    m_current_image.userLabelsRef() = cv::Mat1b(m_calib_data->raw_depth_size);
    m_current_image.userLabelsRef() = 0u;

    if (m_track_users)
        m_current_image.setSkeletonData(new Skeleton());

    bool mapping_required = m_calib_data->rawRgbSize() != m_calib_data->raw_depth_size;
    if (!mapping_required)
    {
        m_rgbd_image.mappedRgbRef() = m_rgbd_image.rawRgbRef();
        m_rgbd_image.mappedDepthRef() = m_rgbd_image.rawDepthRef();
        m_current_image.mappedRgbRef() = m_current_image.rawRgbRef();
        m_current_image.mappedDepthRef() = m_current_image.rawDepthRef();
    }
    else
    {
        m_rgbd_image.mappedRgbRef() = Mat3b(m_calib_data->raw_depth_size);
        m_rgbd_image.mappedRgbRef() = Vec3b(0,0,0);
        m_rgbd_image.mappedDepthRef() = Mat1f(m_calib_data->rawRgbSize());
        m_rgbd_image.mappedDepthRef() = 0.f;
        m_current_image.mappedRgbRef() = Mat3b(m_calib_data->rawDepthSize());
        m_current_image.mappedRgbRef() = Vec3b(0,0,0);
        m_current_image.mappedDepthRef() = Mat1f(m_calib_data->rawRgbSize());
        m_current_image.mappedDepthRef() = 0.f;
    }

    xn::SceneMetaData sceneMD;
    xn::DepthMetaData depthMD;
    xn::ImageMetaData rgbMD;
    xn::IRMetaData irMD;

    ImageBayerGRBG bayer_decoder(ImageBayerGRBG::EdgeAware);

    RGBDImage oversampled_image;
    if (m_subsampling_factor != 1)
    {
        oversampled_image.rawDepthRef().create(m_calib_data->rawDepthSize()*m_subsampling_factor);
        oversampled_image.userLabelsRef().create(oversampled_image.rawDepth().size());
    }

    while (!m_should_exit)
    {
        waitForNewEvent();
        ntk_dbg(2) << format("[%x] running iteration", this);

        {
            // OpenNI calls do not seem to be thread safe.
            QMutexLocker ni_locker(&m_ni_mutex);
            waitAndUpdateActiveGenerators();
        }

        if (m_track_users && m_body_event_detector)
            m_body_event_detector->update();

        m_ni_depth_generator.GetMetaData(depthMD);
        if (m_get_infrared)
        {
            m_ni_ir_generator.GetMetaData(irMD);
        }
        else
        {
            m_ni_rgb_generator.GetMetaData(rgbMD);
        }

        RGBDImage& temp_image =
                m_subsampling_factor == 1 ? m_current_image : oversampled_image;

        const XnDepthPixel* pDepth = depthMD.Data();
        ntk_assert((depthMD.XRes() == temp_image.rawDepth().cols)
                   && (depthMD.YRes() == temp_image.rawDepth().rows),
                   "Invalid image size.");

        // Convert to meters.
        const float depth_correction_factor = 1.0;
        float* raw_depth_ptr = temp_image.rawDepthRef().ptr<float>();
        for (int i = 0; i < depthMD.XRes()*depthMD.YRes(); ++i)
            raw_depth_ptr[i] = depth_correction_factor * pDepth[i]/1000.f;

        if (m_get_infrared)
        {
            const XnGrayscale16Pixel* pImage = irMD.Data();
            m_current_image.rawIntensityRef().create(irMD.YRes(), irMD.XRes());
            float* raw_img_ptr = m_current_image.rawIntensityRef().ptr<float>();
            for (int i = 0; i < irMD.XRes()*irMD.YRes(); ++i)
            {
                raw_img_ptr[i] = pImage[i];
            }
        }
        else
        {
            if (m_custom_bayer_decoding)
            {
                uchar* raw_rgb_ptr = m_current_image.rawRgbRef().ptr<uchar>();
                bayer_decoder.fillRGB(rgbMD,
                                      m_current_image.rawRgb().cols, m_current_image.rawRgb().rows,
                                      raw_rgb_ptr);
                cvtColor(m_current_image.rawRgbRef(), m_current_image.rawRgbRef(), CV_RGB2BGR);
            }
            else
            {
                const XnUInt8* pImage = rgbMD.Data();
                ntk_assert(rgbMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24, "Invalid RGB format.");
                uchar* raw_rgb_ptr = m_current_image.rawRgbRef().ptr<uchar>();
                for (int i = 0; i < rgbMD.XRes()*rgbMD.YRes()*3; i += 3)
                    for (int k = 0; k < 3; ++k)
                    {
                        raw_rgb_ptr[i+k] = pImage[i+(2-k)];
                    }
            }
        }

        if (m_track_users)
        {
            m_ni_user_generator.GetUserPixels(0, sceneMD);
            uchar* user_mask_ptr = temp_image.userLabelsRef().ptr<uchar>();
            const XnLabel* pLabel = sceneMD.Data();
            for (int i = 0; i < sceneMD.XRes()*sceneMD.YRes(); ++i)
            {
                user_mask_ptr[i] = pLabel[i];
            }

            XnUserID user_ids[15];
            XnUInt16 num_users = 15;
            m_ni_user_generator.GetUsers(user_ids, num_users);

            // FIXME: only one user supported.
            for (int i = 0; i < num_users; ++i)
            {
                XnUserID user_id = user_ids[i];
                if (m_ni_user_generator.GetSkeletonCap().IsTracking(user_id))
                {
                    m_current_image.skeletonRef()->computeJoints(user_id, m_ni_user_generator, m_ni_depth_generator);
                    break;
                }
            }
        }

        if (m_subsampling_factor != 1)
        {
            // Cannot use interpolation here, since this would
            // spread the invalid depth values.
            cv::resize(oversampled_image.rawDepth(),
                       m_current_image.rawDepthRef(),
                       m_current_image.rawDepth().size(),
                       0, 0, INTER_NEAREST);
            // we have to repeat this, since resize can change the pointer.
            // m_current_image.depthRef() = m_current_image.rawDepthRef();
            cv::resize(oversampled_image.userLabels(),
                       m_current_image.userLabelsRef(),
                       m_current_image.userLabels().size(),
                       0, 0, INTER_NEAREST);
        }

        {
            QWriteLocker locker(&m_lock);
            m_current_image.swap(m_rgbd_image);
        }

        advertiseNewFrame();
    }
    ntk_dbg(1) << format("[%x] finishing", this);
}

// Callback: New user was detected
void OpenniGrabber :: newUserCallback(XnUserID nId)
{
    ntk_dbg(1) << cv::format("New User %d\n", nId);

    if (m_ni_user_generator.GetNumberOfUsers() > m_max_num_users)
        return;

    if (m_need_pose_to_calibrate)
    {
        m_ni_user_generator.GetPoseDetectionCap().StartPoseDetection(m_calibration_pose, nId);
    }
    else
    {
        m_ni_user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

// Callback: An existing user was lost
void OpenniGrabber :: lostUserCallback(XnUserID nId)
{
    ntk_dbg(1) << cv::format("Lost User %d\n", nId);
}

// Callback: Detected a pose
void OpenniGrabber :: userPoseDetectedCallback(XnUserID nId)
{
    ntk_dbg(1) << cv::format("User Pose Detected %d\n", nId);
    m_ni_user_generator.GetPoseDetectionCap().StopPoseDetection(nId);
    m_ni_user_generator.GetSkeletonCap().RequestCalibration(nId, true);
}

// Callback: Calibration started
void OpenniGrabber :: calibrationStartedCallback(XnUserID nId)
{
    ntk_dbg(1) << cv::format("Calibration started %d\n", nId);
}

// Callback: Finished calibration
void OpenniGrabber :: calibrationFinishedCallback(XnUserID nId, bool success)
{
    ntk_dbg(1) << cv::format("Calibration finished %d\n", nId);
    if (success)
    {
        ntk_dbg(1) << cv::format("Calibration complete, start tracking user %d\n", nId);
        m_ni_user_generator.GetSkeletonCap().StartTracking(nId);
        return;
    }

    ntk_dbg(1) << cv::format("Calibration failed for user %d\n", nId);
    if (m_need_pose_to_calibrate)
    {
        m_ni_user_generator.GetPoseDetectionCap().StartPoseDetection(m_calibration_pose, nId);
    }
    else
    {
        m_ni_user_generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

} // ntk

struct ntk::OpenniDriver::Config
{
    Config (OpenniDriver* that)
        : that(that)
    {
        if(!configFile.open() || !modulesFile.open())
        {
            // Device full?
            assert(false);
            return;
        }

        configFile.write(configText);
        configFile.flush();

        modulesFile.write(modulesText);
        modulesFile.flush();

        modulesPath = modulesFile.fileName().toStdString();

        xn_modules_file = modulesPath.c_str();
    }

    static const char* configText;
    static const char* modulesText;

    QTemporaryFile configFile;
    QTemporaryFile modulesFile;
    std::string modulesPath;
    OpenniDriver* that;
};

const char*
ntk::OpenniDriver::Config::configText = "\
<OpenNI>\n\
<Licenses>\n\
<License vendor=\"PrimeSense\" key=\"0KOIk2JeIBYClPWVnMoRKn5cdY4=\" />\n\
</Licenses>\n\
<Log writeToConsole=\"false\" writeToFile=\"true\">\n\
<!-- 0 - Verbose, 1 - Info, 2 - Warning, 3 - Error (default) -->\n\
<LogLevel value=\"0\"/>\n\
<Masks>\n\
<Mask name=\"ALL\" on=\"true\"/>\n\
</Masks>\n\
<Dumps>\n\
</Dumps>\n\
</Log>\n\
<ProductionNodes>\n\
<Node type=\"Image\" name=\"Image1\">\n\
<Configuration>\n\
<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>\n\
<Mirror on=\"true\"/>\n\
</Configuration>\n\
</Node>\n\
<Node type=\"Depth\" name=\"Depth1\">\n\
<Configuration>\n\
<MapOutputMode xRes=\"640\" yRes=\"480\" FPS=\"30\"/>\n\
<Mirror on=\"true\"/>\n\
</Configuration>\n\
</Node>\n\
<Node type=\"User\" />\n\
<Node type=\"Gesture\" />\n\
<Node type=\"Hands\" />\n\
</ProductionNodes>\n\
</OpenNI>\n\
";

#ifdef __APPLE__
const char*
ntk::OpenniDriver::Config::modulesText = "\
<Modules>\n\
<Module path=\"libnimMockNodes.dylib\" />\n\
<Module path=\"libnimCodecs.dylib\" />\n\
<Module path=\"libnimRecorder.dylib\" />\n\
<Module path=\"libXnDevicesSensorV2.dylib\" configDir=\"config\"/>\n\
<Module path=\"libXnDeviceFile.dylib\" configDir=\"config\"/>\n\
<Module path=\"libXnVFeatures.dylib\" configDir=\"config/XnVFeatures\"/>\n\
<Module path=\"libXnVHandGenerator.dylib\" configDir=\"config/XnVHandGenerator\"/>\n\
</Modules>\n\
";
#else
const char*
ntk::OpenniDriver::Config::modulesText = "\
<Modules>\n\
<Module path=\"../lib/libnimMockNodes.so\" />\n\
<Module path=\"../lib/libnimCodecs.so\" />\n\
<Module path=\"../lib/libnimRecorder.so\" />\n\
<Module path=\"../lib/libXnDevicesSensorV2.so\" configDir=\"config\"/>\n\
<Module path=\"../lib/libXnDeviceFile.so\" configDir=\"config\"/>\n\
<Module path=\"../lib/libXnVFeatures.so\" configDir=\"config/XnVFeatures\"/>\n\
<Module path=\"../lib/libXnVHandGenerator.so\" configDir=\"config/XnVHandGenerator\"/>\n\
</Modules>\n\
";
#endif

ntk::OpenniDriver::OpenniDriver() : m_config(new Config(this))
{
    ntk_dbg(1) << "Initializing OpenNI driver";

    if (ntk::ntk_debug_level >= 1)
    {
        xnLogSetFileOutput(true);
        xnLogSetSeverityFilter(XN_LOG_WARNING);
        xnLogSetMaskState("ALL", true);
    }
    if (ntk::ntk_debug_level >= 2)
    {
        xnLogSetConsoleOutput(true);
        xnLogSetSeverityFilter(XN_LOG_VERBOSE);
    }
    xnLogInitSystem();

    XnStatus status = m_ni_context.Init();
    checkXnError(status, "Initialize context");

    xn::NodeInfoList device_node_info_list;
    status = m_ni_context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, device_node_info_list);
    if (status != XN_STATUS_OK && device_node_info_list.Begin () != device_node_info_list.End ())
        ntk_throw_exception(format("enumerating devices failed. Reason: %s", xnGetStatusString(status)));

    for (xn::NodeInfoList::Iterator nodeIt = device_node_info_list.Begin();
         nodeIt != device_node_info_list.End (); ++nodeIt)
    {
        const xn::NodeInfo& deviceInfo = *nodeIt;
        const XnProductionNodeDescription& description = deviceInfo.GetDescription();
        ntk_dbg(1) << format("Found device: vendor %s name %s", description.strVendor, description.strName);
        m_device_nodes.push_back("kinect");
    }

    strcpy(m_license.strVendor, "PrimeSense");
    strcpy(m_license.strKey, "0KOIk2JeIBYClPWVnMoRKn5cdY4=");
    m_ni_context.AddLicense(m_license);
}

ntk::OpenniDriver :: ~OpenniDriver()
{
    m_ni_context.StopGeneratingAll();
    m_ni_context.Release();
    delete m_config;
}

void ntk::OpenniDriver :: checkXnError(const XnStatus& status, const char* what) const
{
    if (status != XN_STATUS_OK)
    {
        ntk_dbg(0) << "[ERROR] " << cv::format("%s failed: %s\n", what, xnGetStatusString(status));
        ntk_throw_exception("Error in OpenniGrabber.");
    }
}
