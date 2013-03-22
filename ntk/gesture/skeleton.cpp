
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
