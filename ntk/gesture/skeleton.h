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

#ifndef NTK_GESTURE_SKELETON_H
#define NTK_GESTURE_SKELETON_H

#include <ntk/core.h>

namespace xn
{
  class UserGenerator;
  class DepthGenerator;
}

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
  };

  /*! Number of available joints. */
  enum { NumJoints = 24 };

  /*! Number of available links. */
  enum { NumLinks = 16 };

  /*! Ordered list of joints */
  static JointName ntkJointList(int i);

  /*! Represent a link between two joints. */
  struct Link { JointName source; JointName dest; };

  /*! List of links. */
  static Link ntkLinkList(int i)
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
  int m_user_id;
};

/*! Ordered list of joints */
inline Skeleton::JointName Skeleton :: ntkJointList(int i)
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
