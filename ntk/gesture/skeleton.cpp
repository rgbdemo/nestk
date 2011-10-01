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

using namespace cv;

namespace ntk
{

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
     m_joints[j] = Point3f(ni_p[j].X, ni_p[j].Y, ni_p[j].Z);
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
