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

#ifndef NTK_GESTURE_SKELETON_H
#define NTK_GESTURE_SKELETON_H

#include <ntk/core.h>

// OpenNI headers include windows.h on windows without preventing
// preprocessor namespace pollution.
// FIXME: Factor this out.
#ifdef _WIN32
#   define NOMINMAX
#   define VC_EXTRALEAN
#endif
#include <XnOpenNI.h>
#ifdef _WIN32
#   undef VC_EXTRALEAN
#   undef NOMINMAX
#endif

#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

// Under certain odd circumstances, qhull/io.h can be incorrectly included
// by XnPlatformWin32.h, dragging True and False as preprocessor macros,
// breaking in turn flann headers.
// See also: ntk/camera/openni_grabber.h
// FIXME: Ensure that such a broken build system is never generated and remove
// this kludge.
#ifdef _WIN32
#   ifdef True
#       undef True
#   endif
#   ifdef False
#       undef False
#   endif
#endif

namespace ntk
{

/*!
 * Store skeleton data related to an user.
 * Note: this class must remain copiable through std::copy.
 */
class Skeleton
{
public:
  /*! List of available joints. */
  enum JointName
  {
    NTK_SKEL_NIL             = -1,

    NTK_SKEL_HEAD			= 0,
    NTK_SKEL_NECK,
    NTK_SKEL_TORSO,
    NTK_SKEL_WAIST,

    NTK_SKEL_LEFT_COLLAR,
    NTK_SKEL_LEFT_SHOULDER,
    NTK_SKEL_LEFT_ELBOW,
    NTK_SKEL_LEFT_WRIST,
    NTK_SKEL_LEFT_HAND,
    NTK_SKEL_LEFT_FINGERTIP,

    NTK_SKEL_RIGHT_COLLAR,
    NTK_SKEL_RIGHT_SHOULDER,
    NTK_SKEL_RIGHT_ELBOW,
    NTK_SKEL_RIGHT_WRIST,
    NTK_SKEL_RIGHT_HAND,
    NTK_SKEL_RIGHT_FINGERTIP,

    NTK_SKEL_LEFT_HIP,
    NTK_SKEL_LEFT_KNEE,
    NTK_SKEL_LEFT_ANKLE,
    NTK_SKEL_LEFT_FOOT,

    NTK_SKEL_RIGHT_HIP,
    NTK_SKEL_RIGHT_KNEE,
    NTK_SKEL_RIGHT_ANKLE,
    NTK_SKEL_RIGHT_FOOT,   

    NTK_SKEL_NUM,
  };

  /*! Number of available joints. */
  enum { NumJoints = NTK_SKEL_NUM };

  /*! Number of available links. */
  enum { NumLinks = 16 };

  /*! Ordered list of joints */
  static const JointName ntkJointList(int i);

  /*! Represent a link between two joints. */
  struct Link { JointName source; JointName dest; };

  /*! List of links. */
  static const Link ntkLinkList(int i)
  {
    static const Link list[NumLinks] =  {
      {NTK_SKEL_HEAD, NTK_SKEL_NECK},
      {NTK_SKEL_NECK, NTK_SKEL_LEFT_SHOULDER},
      {NTK_SKEL_LEFT_SHOULDER, NTK_SKEL_LEFT_ELBOW},
      {NTK_SKEL_LEFT_ELBOW, NTK_SKEL_LEFT_HAND},

      {NTK_SKEL_NECK, NTK_SKEL_RIGHT_SHOULDER},
      {NTK_SKEL_RIGHT_SHOULDER, NTK_SKEL_RIGHT_ELBOW},
      {NTK_SKEL_RIGHT_ELBOW, NTK_SKEL_RIGHT_HAND},

      {NTK_SKEL_LEFT_SHOULDER, NTK_SKEL_TORSO},
      {NTK_SKEL_RIGHT_SHOULDER, NTK_SKEL_TORSO},

      {NTK_SKEL_TORSO, NTK_SKEL_LEFT_HIP},
      {NTK_SKEL_LEFT_HIP, NTK_SKEL_LEFT_KNEE},
      {NTK_SKEL_LEFT_KNEE, NTK_SKEL_LEFT_FOOT},

      {NTK_SKEL_TORSO, NTK_SKEL_RIGHT_HIP},
      {NTK_SKEL_RIGHT_HIP, NTK_SKEL_RIGHT_KNEE},
      {NTK_SKEL_RIGHT_KNEE, NTK_SKEL_RIGHT_FOOT},

      {NTK_SKEL_LEFT_HIP, NTK_SKEL_RIGHT_HIP},
    };
    return list[i];
  }

private:
  /*! Do not use it directly. Use ntkJointList. */
  static const XnSkeletonJoint xnJointList(int i);

public:
  Skeleton()
  {}

public:
  /*! Deep copy the skeleton data. */
  void copyTo(Skeleton& rhs) const;

  /*! Returns the number of joints. */
  int numJoints() const { return NumJoints; }

  /*! Returns the array of joints. */
  const cv::Point3f* allJoints() const { return m_joints; }

  /*! Return the 3D position of a specific joint. */
  const cv::Point3f& getJoint(JointName name) const { return m_joints[name]; }

  /*!
   * Returns the projected 2D position of a specific joint in image coordinates.
   * The returned point is still 3d because z holds the depth.
   */
  const cv::Point3f& getProjectedJoint(JointName name) const { return m_projected_joints[name]; }

  /*! Draw the skeleton as an overlay over a given image. */
  void drawOnImage(cv::Mat& image) const;

public:
  /*! Compute all joints values from OpenNI. Do not use it. */
  void computeJoints(int user_id,
                     xn::UserGenerator& m_ni_user_generator,
                     xn::DepthGenerator& m_ni_depth_generator);

private:
  /*! WARNING: members must be POD to allow deep memcpy. */
  cv::Point3f m_joints[NumJoints];
  cv::Point3f m_projected_joints[NumJoints];
  cv::Point3f m_primary_hand_points;
  cv::Point3f m_projected_primary_hand_points;
  int m_user_id;
};

inline const XnSkeletonJoint Skeleton :: xnJointList(int i)
{
  static const XnSkeletonJoint list[NumJoints] =  {
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

/*! Ordered list of joints */
inline const Skeleton::JointName Skeleton :: ntkJointList(int i)
{
  static const JointName list[NumJoints] =  {
    NTK_SKEL_HEAD,
    NTK_SKEL_NECK,
    NTK_SKEL_TORSO,
    NTK_SKEL_WAIST,
    NTK_SKEL_LEFT_COLLAR,
    NTK_SKEL_LEFT_SHOULDER,
    NTK_SKEL_LEFT_ELBOW,
    NTK_SKEL_LEFT_WRIST,
    NTK_SKEL_LEFT_HAND,
    NTK_SKEL_LEFT_FINGERTIP,

    NTK_SKEL_RIGHT_COLLAR,
    NTK_SKEL_RIGHT_SHOULDER,
    NTK_SKEL_RIGHT_ELBOW,
    NTK_SKEL_RIGHT_WRIST,
    NTK_SKEL_RIGHT_HAND,
    NTK_SKEL_RIGHT_FINGERTIP,

    NTK_SKEL_LEFT_HIP,
    NTK_SKEL_LEFT_KNEE,
    NTK_SKEL_LEFT_ANKLE,
    NTK_SKEL_LEFT_FOOT,

    NTK_SKEL_RIGHT_HIP,
    NTK_SKEL_RIGHT_KNEE,
    NTK_SKEL_RIGHT_ANKLE,
    NTK_SKEL_RIGHT_FOOT
  };
  return list[i];
}

} // ntk

#endif // NTK_GESTURE_SKELETON_H
