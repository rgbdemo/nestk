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

#include "nite_rgbd_grabber.h"

#include <ntk/utils/opencv_utils.h>
#include <ntk/gesture/body_event.h>
#include <ntk/geometry/pose_3d.h>
#include <XnVCircleDetector.h>

using namespace cv;
using namespace ntk;

namespace
{

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
  ntk::NiteRGBDGrabber* grabber = reinterpret_cast<ntk::NiteRGBDGrabber*>(pCookie);
  grabber->calibrationStartedCallback(nId);
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ntk::NiteRGBDGrabber* grabber = reinterpret_cast<ntk::NiteRGBDGrabber*>(pCookie);
  grabber->newUserCallback(nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  ntk::NiteRGBDGrabber* grabber = reinterpret_cast<ntk::NiteRGBDGrabber*>(pCookie);
  grabber->lostUserCallback(nId);
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
  ntk::NiteRGBDGrabber* grabber = reinterpret_cast<ntk::NiteRGBDGrabber*>(pCookie);
  grabber->userPoseDetectedCallback(nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
  ntk::NiteRGBDGrabber* grabber = reinterpret_cast<ntk::NiteRGBDGrabber*>(pCookie);
  grabber->calibrationFinishedCallback(nId, bSuccess);
}

} // namespace anonymous

namespace ntk
{

void NiteRGBDGrabber :: check_error(const XnStatus& status, const char* what) const
{
  if (status != XN_STATUS_OK)
  {
    ntk_dbg(0) << "[ERROR] " << cv::format("%s failed: %s\n", what, xnGetStatusString(status));
    ntk_throw_exception("Error in NiteRGBDGrabber.");
  }
}

void NiteRGBDGrabber :: initialize()
{
  const char* config_file = "config/NestkConfig.xml";

  xn::EnumerationErrors errors;
  XnStatus status = m_ni_context.InitFromXmlFile(config_file, &errors);
  if (status != XN_STATUS_OK)
  {
    ntk_dbg(0) << "[ERROR] " << xnGetStatusString(status);
    ntk_throw_exception("Could not initialize NITE. Check Log."
                        "Most probable reasons are device not connected or no config/ directory"
                        "in the current directory.");
  }

  status = m_ni_context.FindExistingNode(XN_NODE_TYPE_DEPTH, m_ni_depth_generator);
  check_error(status, "Find depth generator");

  status = m_ni_context.FindExistingNode(XN_NODE_TYPE_USER, m_ni_user_generator);
  check_error(status, "Find user generator");

  status = m_ni_context.FindExistingNode(XN_NODE_TYPE_IMAGE, m_ni_rgb_generator);
  check_error(status, "Find image generator");

  ntk_ensure(m_ni_depth_generator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT), "Cannot register images.");
  m_ni_depth_generator.GetAlternativeViewPointCap().SetViewPoint(m_ni_rgb_generator);

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
    m_body_event_detector->initialize(m_ni_context);

  status = m_ni_context.StartGeneratingAll();
  check_error(status, "StartGenerating");

  m_ni_context.WaitAndUpdateAll();
  estimateCalibration();
}

void NiteRGBDGrabber :: estimateCalibration()
{
  XnPoint3D p;
  p.X = 0; p.Y = 0; p.Z = -1;
  m_ni_depth_generator.ConvertProjectiveToRealWorld(1, &p, &p);
  ntk_dbg_print(p.X, 1);
  ntk_dbg_print(p.Y, 1);
  ntk_dbg_print(p.Z, 1);

  p.X = 0; p.Y = 0; p.Z = -1;
  m_ni_depth_generator.ConvertRealWorldToProjective(1, &p, &p);
  double cx = p.X;
  double cy = p.Y;

  p.X = 1; p.Y = 1; p.Z = -1;
  m_ni_depth_generator.ConvertRealWorldToProjective(1, &p, &p);

  double fx = -(p.X-cx);
  double fy = p.Y-cy;
  ntk_dbg_print(fx, 1);
  ntk_dbg_print(fy, 1);

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

  m_calib_data->rgb_intrinsics = cv::Mat1d(3,3);
  setIdentity(m_calib_data->rgb_intrinsics);
  m_calib_data->rgb_intrinsics(0,0) = fx;
  m_calib_data->rgb_intrinsics(1,1) = fy;
  m_calib_data->rgb_intrinsics(0,2) = cx;
  m_calib_data->rgb_intrinsics(1,2) = cy;

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
}

void NiteRGBDGrabber :: run()
{
  m_should_exit = false;
  m_current_image.setCalibration(m_calib_data);
  m_rgbd_image.setCalibration(m_calib_data);

  m_rgbd_image.rawRgbRef() = Mat3b(480, 640);
  m_rgbd_image.rawDepthRef() = Mat1f(480, 640);
  m_rgbd_image.rawIntensityRef() = Mat1f(480, 640);
  m_rgbd_image.rgbRef() = m_rgbd_image.rawRgbRef();
  m_rgbd_image.depthRef() = m_rgbd_image.rawDepthRef();
  m_rgbd_image.intensityRef() = m_rgbd_image.rawIntensityRef();
  m_rgbd_image.mappedRgbRef() = m_rgbd_image.rawRgbRef();
  m_rgbd_image.mappedDepthRef() = m_rgbd_image.rawDepthRef();

  m_current_image.rawRgbRef() = Mat3b(480, 640);
  m_current_image.rawDepthRef() = Mat1f(480, 640);
  m_current_image.rawIntensityRef() = Mat1f(480, 640);
  m_current_image.rgbRef() = m_current_image.rawRgbRef();
  m_current_image.depthRef() = m_current_image.rawDepthRef();
  m_current_image.intensityRef() = m_current_image.rawIntensityRef();
  m_current_image.mappedRgbRef() = m_rgbd_image.rawRgbRef();
  m_current_image.mappedDepthRef() = m_rgbd_image.rawDepthRef();

  xn::SceneMetaData sceneMD;
  xn::DepthMetaData depthMD;
  xn::ImageMetaData rgbMD;

  while (!m_should_exit)
  {
    waitForNewEvent();
    m_ni_context.WaitAndUpdateAll();
    if (m_body_event_detector)
      m_body_event_detector->update();
    m_ni_depth_generator.GetMetaData(depthMD);
    m_ni_rgb_generator.GetMetaData(rgbMD);
    m_ni_user_generator.GetUserPixels(0, sceneMD);

    const XnDepthPixel* pDepth = depthMD.Data();
    ntk_assert((depthMD.XRes() == m_current_image.rawDepth().cols)
               && (depthMD.YRes() == m_current_image.rawDepth().rows),
               "Invalid image size.");
    float* raw_depth_ptr = m_current_image.rawDepthRef().ptr<float>();
    for (int i = 0; i < depthMD.XRes()*depthMD.YRes(); ++i)
      raw_depth_ptr[i] = pDepth[i]/1000.f;

    float d = m_current_image.rawDepth()(250,240);
    Point3f p1 = m_calib_data->depth_pose->unprojectFromImage(Point2f(240,250), d);
    ntk_dbg_print(p1, 2);

    XnPoint3D p; p.X = 240; p.Y = 250; p.Z = d;
    m_ni_depth_generator.ConvertProjectiveToRealWorld(1, &p, &p);
    Point3f p2 (p.X, p.Y, p.Z);
    ntk_dbg_print(p2, 2);

    const XnUInt8* pImage = rgbMD.Data();
    ntk_assert(rgbMD.PixelFormat() == XN_PIXEL_FORMAT_RGB24, "Invalid RGB format.");
    uchar* raw_rgb_ptr = m_current_image.rawRgbRef().ptr<uchar>();
    for (int i = 0; i < rgbMD.XRes()*rgbMD.YRes()*3; i += 3)
    for (int k = 0; k < 3; ++k)
    {
      raw_rgb_ptr[i+k] = pImage[i+(2-k)];
    }

    XnUserID user_ids[15];
    XnUInt16 num_users = 15;
    m_ni_user_generator.GetUsers(user_ids, num_users);

    XnUserID user_id = user_ids[0];
    if (m_ni_user_generator.GetSkeletonCap().IsTracking(user_id))
    {
      m_skeleton_data.computeJoints(user_id, m_ni_user_generator, m_ni_depth_generator);
    }

    {
      QWriteLocker locker(&m_lock);
      m_current_image.swap(m_rgbd_image);
    }

    advertiseNewFrame();
  }
}

// Callback: New user was detected
void NiteRGBDGrabber :: newUserCallback(XnUserID nId)
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
void NiteRGBDGrabber :: lostUserCallback(XnUserID nId)
{
  ntk_dbg(1) << cv::format("Lost User %d\n", nId);
}

// Callback: Detected a pose
void NiteRGBDGrabber :: userPoseDetectedCallback(XnUserID nId)
{
  ntk_dbg(1) << cv::format("User Pose Detected %d\n", nId);
  m_ni_user_generator.GetPoseDetectionCap().StopPoseDetection(nId);
  m_ni_user_generator.GetSkeletonCap().RequestCalibration(nId, true);
}

// Callback: Calibration started
void NiteRGBDGrabber :: calibrationStartedCallback(XnUserID nId)
{
  ntk_dbg(1) << cv::format("Calibration started %d\n", nId);
}

// Callback: Finished calibration
void NiteRGBDGrabber :: calibrationFinishedCallback(XnUserID nId, bool success)
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
