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

#include "skeleton.h"

#ifdef NESTK_USE_OPENNI

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnVSessionManager.h>
#include <XnVPushDetector.h>

using namespace cv;

namespace ntk
{

  static XnSkeletonJoint xnJointList(int i)
  {
    static const XnSkeletonJoint list[Skeleton::NumJoints] =  {
    XN_SKEL_HEAD,
    XN_SKEL_NECK,
    XN_SKEL_TORSO,
    XN_SKEL_WAIST,
    XN_SKEL_LEFT_COLLAR,
    XN_SKEL_LEFT_SHOULDER,
    XN_SKEL_LEFT_ELBOW,
    XN_SKEL_LEFT_WRIST,
    XN_SKEL_LEFT_HAND,
    XN_SKEL_LEFT_FINGERTIP,

    XN_SKEL_RIGHT_COLLAR,
    XN_SKEL_RIGHT_SHOULDER,
    XN_SKEL_RIGHT_ELBOW,
    XN_SKEL_RIGHT_WRIST,
    XN_SKEL_RIGHT_HAND,
    XN_SKEL_RIGHT_FINGERTIP,

    XN_SKEL_LEFT_HIP,
    XN_SKEL_LEFT_KNEE,
    XN_SKEL_LEFT_ANKLE,
    XN_SKEL_LEFT_FOOT,

    XN_SKEL_RIGHT_HIP,
    XN_SKEL_RIGHT_KNEE,
    XN_SKEL_RIGHT_ANKLE,
    XN_SKEL_RIGHT_FOOT
    };
    return list[i];
  }

 void Skeleton :: computeJoints(int user_id,
                                        xn::UserGenerator& user_generator,
                                        xn::DepthGenerator& depth_generator)
 {
   XnPoint3D ni_p[NumJoints];
   for (int j = 0; j < NumJoints; ++j)
   {
     XnSkeletonJointPosition joint;
     user_generator.GetSkeletonCap().GetSkeletonJointPosition(user_id,
                                                              xnJointList(j),
                                                              joint);
     ni_p[j] = joint.position;
     m_joints[j] = Point3f(ni_p[0].X, ni_p[0].Y, ni_p[0].Z);
   }

   depth_generator.ConvertRealWorldToProjective(NumJoints, ni_p, ni_p);

   for (int j = 0; j < NumJoints; ++j)
   {
     m_projected_joints[j] = Point3f(ni_p[j].X, ni_p[j].Y, ni_p[j].Z);
   }
 }

 void Skeleton :: drawOnImage(Mat& image) const
 {
   for (int i = 0; i < NumLinks; ++i)
   {
     Point3f p1 = getProjectedJoint(ntkLinkList(i).source);
     Point3f p2 = getProjectedJoint(ntkLinkList(i).dest);
     cv::line(image, Point(p1.x,p1.y), Point(p2.x,p2.y), Scalar(255,255,255,255));
   }
 }

 void Skeleton :: copyTo(Skeleton& rhs) const
 {
   memcpy(&rhs, this, sizeof(Skeleton));
 }

} // ntk

#else // NESTK_USE_OPENNI



#endif // NESTK_USE_OPENNI
