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

#ifndef NESTK_GEOMETRY_EIGEN_UTILS_H
#define NESTK_GEOMETRY_EIGEN_UTILS_H

#include <ntk/core.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/serializable.h>
#include <ntk/geometry/pose_3d.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ntk
{

template < typename _Scalar, int N, int M>
const NtkDebug& operator<<(const NtkDebug& output, const Eigen::Matrix<_Scalar,N,M>& array)
{   
  for (int i = 0; i < N; ++i)
  {
      output << "[";
      for (int j = 0; j < M; ++j)
          output << array(i,j) << " ";
      output << "]";
  }
  return output;
}

template < typename StreamType, typename _Scalar, int N>
StreamType& operator>>(StreamType& input, Eigen::Matrix<_Scalar,N,1>& array)
{
  for (int i = 0; i < N; ++i)
  {
    if (ntk::isStreamCorrupted(input)) ntk_throw_exception("Corrupted stream.");
    input >> array(i);
  }
  return input;
};

template < typename StreamType, typename _Scalar, int N>
StreamType& operator<<(StreamType& output, const Eigen::Matrix<_Scalar,N,1>& array)
{
  for (int i = 0; i < N; ++i)
    output << array(i) << ntk::sep();
  return output;
};

template < typename StreamType, typename _Scalar, int N, int M>
StreamType& operator<<(StreamType& output, const Eigen::Matrix<_Scalar,N,M>& array)
{
  for (int i = 0; i < N; ++i)
      for (int j = 0; j < M; ++j)
    output << array(i,j) << ntk::sep();
  return output;
}

inline Eigen::Vector3f eigen_cross (const Eigen::Vector4f& v1, const Eigen::Vector4f& v2)
{
    Eigen::Vector3f v1_3f (v1.x(), v1.y(), v1.z());
    Eigen::Vector3f v2_3f (v2.x(), v2.y(), v2.z());
    return v1_3f.cross (v2_3f);
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Point3_<CvScalarType>& p, Eigen::Matrix<EScalarType,3,1>& ep)
{
  ep(0) = p.x;
  ep(1) = p.y;
  ep(2) = p.z;
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Vec<CvScalarType,3>& p, Eigen::Matrix<EScalarType,3,1>& ep)
{
  ep(0) = p[0];
  ep(1) = p[1];
  ep(2) = p[2];
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Vec<CvScalarType,3>& p, Eigen::Matrix<EScalarType,4,1>& ep)
{
  ep(0) = p[0];
  ep(1) = p[1];
  ep(2) = p[2];
  ep(3) = 1;
}

template <typename CvScalarType, typename EScalarType>
inline void toEigen(const cv::Point3_<CvScalarType>& p, Eigen::Matrix<EScalarType,4,1>& ep)
{
  ep(0) = p.x;
  ep(1) = p.y;
  ep(2) = p.z;
  ep(3) = 1;
}

#ifdef _MSC_VER
inline void toOpencv(const Eigen::Matrix4d& ep,
                     cv::Mat1f& mat)
{
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r,c) = ep(r,c);
}
inline void toOpencv(const Eigen::Matrix4f& ep,
                     cv::Mat1f& mat)
{
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r,c) = ep(r,c);
}
inline void toOpencv(const Eigen::Matrix4d& ep,
                     cv::Mat1d& mat)
{
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r,c) = ep(r,c);
}
inline void toOpencv(const Eigen::Matrix4f& ep,
                     cv::Mat1d& mat)
{
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      mat(r,c) = ep(r,c);
}
#else
template <typename CvScalarType, typename EigenScalarType, int H, int W>
inline void toOpencv(const Eigen::Matrix<EigenScalarType,H,W>& ep,
                         cv::Mat_<CvScalarType>& mat)
{
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c)
      mat(r,c) = ep(r,c);
}
#endif

inline ntk::Pose3D toPose3D(const Eigen::Matrix4f& m)
{
    cv::Mat1f cv_pose(4,4);
    toOpencv(m, cv_pose);
    ntk::Pose3D pose;
    pose.setCameraTransform(cv_pose);
    return pose;
}

// FIXME: MSVC10 ICEs on the generic function definition.
#ifdef _MSC_VER
#define TO_EIGEN_MSVC_DEF(H,W)  \
{                               \
  for (int r = 0; r < H; ++r)   \
    for (int c = 0; c < W; ++c) \
      ep(r,c) = mat(r,c);       \
}
inline void toEigen(const cv::Mat1f& mat, Eigen::Matrix<float, 4, 4>& ep) TO_EIGEN_MSVC_DEF(4,4)
inline void toEigen(const cv::Mat1d& mat, Eigen::Matrix<double, 4, 4>& ep) TO_EIGEN_MSVC_DEF(4,4)
inline void toEigen(const cv::Mat1f& mat, Eigen::Matrix<float, 3, 3>& ep) TO_EIGEN_MSVC_DEF(3,3)
inline void toEigen(const cv::Mat1d& mat, Eigen::Matrix<double, 3, 3>& ep) TO_EIGEN_MSVC_DEF(3,3)
#undef TO_EIGEN_MSVC_DEF
#else
template <typename CvScalarType, typename EScalarType, int H, int W>
inline void toEigen(const cv::Mat_<CvScalarType>& mat, Eigen::Matrix<EScalarType,H,W>& ep)
{
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c)
      ep(r,c) = mat(r,c);
}
#endif

template <typename EScalarType>
inline cv::Vec3f toVec3f(const Eigen::Matrix<EScalarType,3,1>& v)
{
  return cv::Vec3f(v(0),v(1),v(2));
}

template <typename EScalarType>
inline cv::Vec3f toVec3f(const Eigen::Matrix<EScalarType,4,1>& v)
{
  // FIXME: divide by v(3) ?
  return cv::Vec3f(v(0),v(1),v(2));
}

inline Eigen::Vector2d toEigenVector2d(const cv::Point2f& v)
{ Eigen::Vector2d r (v.x, v.y); return r; }

inline Eigen::Vector3d toEigenVector3d(const cv::Vec3f& v)
{ Eigen::Vector3d r; toEigen(v, r); return r; }

inline Eigen::Vector3f toEigenVector3f(const cv::Vec3f& v)
{ Eigen::Vector3f r; toEigen(v, r); return r; }

inline Eigen::Vector3f toEigenVector3f(const Eigen::Vector4f& v)
{ return Eigen::Map<const Eigen::Vector3f, Eigen::Aligned>(v.data()); }

inline Eigen::Map<const Eigen::Vector3f, Eigen::Aligned> asEigenVector3f(const Eigen::Vector4f& v)
{ return Eigen::Map<const Eigen::Vector3f, Eigen::Aligned>(v.data()); }

inline Eigen::Vector4f toEigenVector4f(const cv::Vec3f& v)
{ Eigen::Vector4f r (v[0], v[1], v[2], 1); return r; }

inline Eigen::Vector4d toEigenVector4d(const cv::Vec3f& v)
{ Eigen::Vector4d r; toEigen(v, r); return r; }

template<typename _Scalar> class EulerAngles
{
public:
  enum { Dim = 3 };
  typedef _Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,3> Matrix3;
  typedef Eigen::Matrix<Scalar,3,1> Vector3;
  typedef Eigen::Quaternion<Scalar> QuaternionType;

protected:

  Vector3 m_angles;

public:

  EulerAngles() {}
  inline EulerAngles(Scalar a0, Scalar a1, Scalar a2) : m_angles(a0, a1, a2) {}
  inline EulerAngles(const QuaternionType& q) { *this = q; }

  const Vector3& coeffs() const { return m_angles; }
  Vector3& coeffs() { return m_angles; }

  EulerAngles& operator=(const QuaternionType& q)
  {
    Matrix3 m = q.toRotationMatrix();
    return *this = m;
  }

  EulerAngles& operator=(const Matrix3& m)
  {
    // mat =  cy*cz          -cy*sz           sy
    //        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
    //       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
    m_angles.coeffRef(1) = std::asin(m.coeff(0,2));
    m_angles.coeffRef(0) = std::atan2(-m.coeff(1,2),m.coeff(2,2));
    m_angles.coeffRef(2) = std::atan2(-m.coeff(0,1),m.coeff(0,0));
    return *this;
  }

  Matrix3 toRotationMatrix(void) const
  {
    Vector3 c = m_angles.array().cos();
    Vector3 s = m_angles.array().sin();
    Matrix3 res;
    res <<  c.y()*c.z(),                    -c.y()*s.z(),                   s.y(),
        c.z()*s.x()*s.y()+c.x()*s.z(),  c.x()*c.z()-s.x()*s.y()*s.z(),  -c.y()*s.x(),
        -c.x()*c.z()*s.y()+s.x()*s.z(), c.z()*s.x()+c.x()*s.y()*s.z(),  c.x()*c.y();
    return res;
  }

  operator QuaternionType() { return QuaternionType(toRotationMatrix()); }
};

} // ntk

#endif // NESTK_GEOMETRY_EIGEN_UTILS_H
