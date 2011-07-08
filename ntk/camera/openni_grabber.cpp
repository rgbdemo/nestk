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

namespace ntk
{

void OpenniGrabber :: check_error(const XnStatus& status, const char* what) const
{
    if (status != XN_STATUS_OK)
    {
        ntk_dbg(0) << "[ERROR] " << cv::format("%s failed: %s\n", what, xnGetStatusString(status));
        ntk_throw_exception("Error in OpenniGrabber.");
    }
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

    xnLogSetConsoleOutput(true);
    xnLogSetSeverityFilter(XN_LOG_WARNING);
    xnLogSetMaskState("LOG", true);
    xnLogInitSystem();

    XnStatus status = m_ni_context.Init();
    check_error(status, "Initialize context");

    xn::NodeInfoList device_node_info_list;
    status = m_ni_context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, device_node_info_list);
    if (status != XN_STATUS_OK && device_node_info_list.Begin () != device_node_info_list.End ())
        ntk_throw_exception(format("enumerating devices failed. Reason: %s", xnGetStatusString(status)));

    // No device
    if (device_node_info_list.IsEmpty())
        ntk_throw_exception(format("No device connected.\n"));

    for (xn::NodeInfoList::Iterator nodeIt = device_node_info_list.Begin();
         nodeIt != device_node_info_list.End (); ++nodeIt)
    {
        const xn::NodeInfo& deviceInfo = *nodeIt;
        const XnProductionNodeDescription& description = deviceInfo.GetDescription();
        ntk_dbg(1) << format("device: vendor %s name %s, instance %s",
                             description.strVendor, description.strName, deviceInfo.GetInstanceName());
    }

    xn::NodeInfoList::Iterator nodeIt = device_node_info_list.Begin();
    for (int i = 0; nodeIt != device_node_info_list.End () && i < m_camera_id; ++nodeIt, ++i)
    {
    }

    if (nodeIt == device_node_info_list.End())
    {
        ntk_throw_exception(format("No device with id %d\n", m_camera_id));
    }

    xn::NodeInfo deviceInfo = *nodeIt;
    status = m_ni_context.CreateProductionTree(deviceInfo);
    check_error(status, "Create Device");

    const XnProductionNodeDescription& description = deviceInfo.GetDescription();
    ntk_dbg(1) << format("device: vendor %s name %s, instance %s\n",
                         description.strVendor, description.strName, deviceInfo.GetInstanceName());

    xn::Query query;
    query.AddNeededNode(deviceInfo.GetInstanceName());

    strcpy(license.strVendor, "PrimeSense");
    strcpy(license.strKey, "0KOIk2JeIBYClPWVnMoRKn5cdY4=");
    m_ni_context.AddLicense(license);

    check_error(status, "Create depth generator");
    status = m_ni_depth_generator.Create(m_ni_context, &query);
    XnMapOutputMode depth_mode;
    depth_mode.nXRes = 640;
    depth_mode.nYRes = 480;
    depth_mode.nFPS = 30;
    m_ni_depth_generator.SetMapOutputMode(depth_mode);

    status = m_ni_rgb_generator.Create(m_ni_context, &query);
    check_error(status, "Create image generator");

    status = m_ni_rgb_generator.SetIntProperty ("Resolution", 1);
    check_error(status, "Resolution");

    status = m_ni_context.SetGlobalMirror(m_mirrored);
    check_error(status, "Mirror");

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

    ntk_ensure(m_ni_depth_generator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT), "Cannot register images.");
    m_ni_depth_generator.GetAlternativeViewPointCap().SetViewPoint(m_ni_rgb_generator);

    if (m_track_users)
    {
        status = m_ni_user_generator.Create(m_ni_context, &query);
        check_error(status, "Create user generator");
        status = m_ni_hands_generator.Create(m_ni_context, &query);
        check_error(status, "Create hands generator");
        status = m_ni_gesture_generator.Create(m_ni_context, &query);
        check_error(status, "Create gestures generator");
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
            m_body_event_detector->initialize(m_ni_context, m_ni_depth_generator);
    }

    status = m_ni_context.StartGeneratingAll();
    check_error(status, "StartGenerating");

    if (m_custom_bayer_decoding)
    {
        // Grayscale to get raw Bayer pattern.
        status = m_ni_rgb_generator.SetIntProperty ("InputFormat", 6);
        check_error(status, "Change input format");

        status = m_ni_rgb_generator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
        check_error(status, "Change pixel format");
    }

    m_ni_context.WaitAndUpdateAll();
    if (!m_calib_data)
        estimateCalibration();
    return true;
}

bool OpenniGrabber :: disconnectFromDevice()
{
    QMutexLocker ni_locker(&m_ni_mutex);
    ntk_dbg(1) << format("[Kinect %x] disconnecting", this);
    if (m_body_event_detector)
        m_body_event_detector->shutDown();
    m_ni_context.Shutdown();
	return true;
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

    m_calib_data = new RGBDCalibration();

    xn::DepthMetaData depthMD;
    m_ni_depth_generator.GetMetaData(depthMD);

    xn::ImageMetaData rgbMD;
    m_ni_rgb_generator.GetMetaData(rgbMD);

    int depth_width = depthMD.XRes();
    int depth_height = depthMD.YRes();

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
        m_current_image.mappedRgbRef() = Mat3b(m_calib_data->raw_depth_size);
        m_current_image.mappedRgbRef() = Vec3b(0,0,0);
        m_current_image.mappedDepthRef() = Mat1f(m_calib_data->rawRgbSize());
        m_current_image.mappedDepthRef() = 0.f;
    }

    xn::SceneMetaData sceneMD;
    xn::DepthMetaData depthMD;
    xn::ImageMetaData rgbMD;

    ImageBayerGRBG bayer_decoder(ImageBayerGRBG::EdgeAware);

    while (!m_should_exit)
    {
        waitForNewEvent();
        ntk_dbg(2) << format("[%x] running iteration", this);

        // OpenNI calls do not seem to be thread safe.
        {
            QMutexLocker ni_locker(&m_ni_mutex);
            m_ni_context.WaitAndUpdateAll();
        }

        if (m_track_users && m_body_event_detector)
            m_body_event_detector->update();

        m_ni_depth_generator.GetMetaData(depthMD);
        m_ni_rgb_generator.GetMetaData(rgbMD);

        const XnDepthPixel* pDepth = depthMD.Data();
        ntk_assert((depthMD.XRes() == m_current_image.rawDepth().cols)
                   && (depthMD.YRes() == m_current_image.rawDepth().rows),
                   "Invalid image size.");

        // Convert to meters.
        const float depth_correction_factor = 1.0;
        float* raw_depth_ptr = m_current_image.rawDepthRef().ptr<float>();
        for (int i = 0; i < depthMD.XRes()*depthMD.YRes(); ++i)
            raw_depth_ptr[i] = depth_correction_factor * pDepth[i]/1000.f;

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

        if (m_track_users)
        {
            m_ni_user_generator.GetUserPixels(0, sceneMD);
            uchar* user_mask_ptr = m_current_image.userLabelsRef().ptr<uchar>();
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
