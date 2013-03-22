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

#ifndef NTK_GEOMETRY_POSE3D_H
#define NTK_GEOMETRY_POSE3D_H

# include <ntk/core.h>
# include <ntk/utils/xml_serializable.h>
# include <ntk/numeric/utils.h>
# include <QString>

namespace ntk
{

// Use Eigen holder to avoid exposing Eigen API here and slow down
// compilation times.
class EigenIsometry3dHolder;

/*!
 * Represent transformations within a Pin-Hole or orthographic camera model.
 * Can be used both for 3D camera transforms and projections
 * on an image plane using perspective or othogonal model.
 * The default representation is OpenGL-like:
 *   Y
 *   |__ X
 *  /
 * Z
 */
class Pose3D : public ntk::XmlSerializable
{
private:
  class PrivatePose3D;
  friend class PrivatePose3D;
  PrivatePose3D* impl;

public:
  Pose3D();
  ~Pose3D();

  Pose3D(const Pose3D& rhs);
  Pose3D& operator=(const Pose3D& rhs);
  Pose3D& swap(Pose3D& other);

public:
  virtual void fillXmlElement(XMLNode& element) const;
  virtual void loadFromXmlElement(const XMLNode& element);
  virtual void saveToYaml(cv::FileStorage& yaml) const;
  virtual void loadFromYaml(cv::FileNode yaml);
  virtual QString toString() const;

  /*! Parse a blender generated camera transform. */
  void parseBlenderFile(const char* filename, int image_width, int image_height);

  /*! Save to a bundler compatible file. */
  void saveToBundlerFile(const char* filename);

  /*! Parse/Save to an AVS file. */
  void parseAvsFile(const char* filename);
  void saveToAvsFile(const char* filename);

  /*! Initialize from Blender camera paramters. */
  void loadFromBlenderParameters(double tx, double ty, double tz,
                                 double rx, double ry, double rz,
                                 double field_of_view,
                                 int image_width,
                                 int image_height);

  /*! Transform into Blender camera parameters. */
  void toBlenderParameters(int image_width, int image_height,
                           double* tx, double* ty, double* tz,
                           double* rx, double* ry, double* rz,
                           double* field_of_view) const;

  /*! Set parameters from intrinsics matrix. */
  void setCameraParametersFromOpencv(const cv::Mat1d& cv_matrix);

  /*!  Set parameters from intrinsics matrix. */
  void setCameraParameters(double fx, double fy, double cx, double cy, bool orthographic = false);

  /*! Adjust intrinsics to match the given subsampling factor. */
  void subsampleImageSize (int subsampling_factor);

  /*! Adjust intrinsics to match the given upsampling factor. */
  void upsampleImageSize(int upsampling_factor);

public:
  /*!
   * Transform a right camera into a left camera using stereo parameters.
   * \param intrinsics_matrix intrinsics matrix of the left camera.
   * \param R extrinsics 3x3 rotation matrix.
   * \param T extrinsics 1x3 translation matrix.
   * @see RGBDCalibration
   */
  void toLeftCamera(const cv::Mat1d& intrinsics_matrix,
                    const cv::Mat1d& R,
                    const cv::Mat1d& T);

  /*!
   * Transform a left camera into a right camera using stereo parameters.
   * @see toLeftCamera
   */
  void toRightCamera(const cv::Mat1d& cv_matrix,
                     const cv::Mat1d& R,
                     const cv::Mat1d& T);

  /*! Compute the delta camera transform with another pose. */
  Pose3D computeDeltaPoseWith(const Pose3D& new_pose) const;

public:
  /*! Focal lenghts in pixels. */
  double focalX() const { return m_focal_x; }
  double focalY() const { return m_focal_y; }

  /*! Image plane center. */
  double imageCenterX() const { return m_image_center_x; }
  double imageCenterY() const { return m_image_center_y; }

  /*! Whether pixels have square size. */
  bool focalAreIdentical() const { 
    return ntk::flt_eq(static_cast<float>(m_focal_x), static_cast<float>(m_focal_y), 1e-5f); 
  }

  /*! Mean focal. */
  double meanFocal() const { return (m_focal_x + m_focal_y)/2.0; }

  /*! Whether it can be used as a complete camera model. */
  bool isValid() const { return m_has_camera_params; }

  /*! Whether the transform is identity. */
  bool isIdentity(float rotation_cos_threshold = 0.999999f, float translation_threshold = 1e-5f * 1e-5f) const;

  /*! Change from pin-hole to orthographic model. */
  void setOrthographic(bool ortho);
  bool isOrthographic() const { return m_orthographic; }

  /*! Return the determinant of the projection matrix. */
  float determinant() const;

public:
  /*! Returns the camera translation as OpenCV 3x3 matrix. */
  const cv::Vec3f cvTranslation() const;

  /*!
   * Returns the camera rotation as OpenCV vector of euler angles.
   * First angle is around X, second around Y and third around Z.
   */
  const cv::Vec3f cvEulerRotation() const;

  /*! Returns the camera rotation as a rodrigues vector. */
  const cv::Vec3f cvRodriguesRotation() const;

  /*! Returns the rotation as a quaternion. */
  cv::Vec4f cvQuaternionRotation() const;

  /*! Returns the camera transform as an OpenCV float 4x4 matrix. */
  const cv::Mat1f cvCameraTransform() const;

  /*! Returns the camera transform as an OpenCV double 4x4 matrix. */
  const cv::Mat1d cvCameraTransformd() const;

  /*! Returns the camera transform as an Eigen double 4x4 matrix. */
  void getEigenCameraTransform(EigenIsometry3dHolder* holder) const;

  /*!
   * Returns the camera transform as an OpenCV 3x3 rotation matrix
   * and a translation vector as a 3x1 matrix.
   * Useful to update calibration parameters.
   */
  void cvRotationMatrixTranslation(cv::Mat1d& translation, cv::Mat1d& rotation) const;

  /*! Returns the inverse camera transform as an OpenCV 4x4 matrix. */
  cv::Mat1f cvInvCameraTransform() const;

  /*! Returns the 4x4 projection matrix (intrinsics * camera) */
  cv::Mat1f cvProjectionMatrix() const;

  /*! Returns the 4x4 inverse projection matrix from image to plane. */
  cv::Mat1f cvInvProjectionMatrix() const;

public:
  /*! Reset the camera transform to Identity. */
  void resetCameraTransform();

  /*! Invert the camera transform. */
  void invert();

  /*! Return the inverted transform. */
  Pose3D inverted() const;

  /*! Go from OpenGL coordinate system to ROS coordinate system. */
  void fromGLToRos();

  /*! Go from OpenCV coordinate system (x right, y down) to OpenGL (x right, y up). */
  void fromCvToGL();

  /*! Go from OpenCV coordinate system (x right, y down) to OpenGL (x right, y up). */
  void applyFromCvToGLBefore();

  /*!
   * Set the 3D camera transformation from OpenCV translation
   * and rodrigues vector.
   */
  void setCameraTransform(const cv::Mat1d& tvec, const cv::Mat1d& rvec);

  /*! Set the 3D camera transform from 4x4 matrix. */
  void setCameraTransform(const cv::Mat1f& H);

  /*! Set the 3D camera transform from 4x4 matrix. */
  void setCameraTransform(const cv::Mat1d& H);

  /*! Set the 3D camera transform from another pose. */
  void setCameraTransform(const Pose3D& pose);

  /*! Set the 3D camera transform from 3x3 fundamental matrix. */
  void setCameraTransformFromCvFundamentalMatrix(const cv::Mat1f& F);

  /*! Apply a camera transform on the right. */
  void applyTransformBefore(const Pose3D& rhs_pose);
  void applyTransformBefore(const cv::Vec3f& cvTranslation, const cv::Vec3f& rotation_euler_angles);
  void applyTransformBefore(const cv::Vec3f& cvTranslation, const cv::Mat1d& rotation_matrix);
  void applyTransformBeforeRodrigues(const cv::Vec3f& cvTranslation, const cv::Vec3f& rotation_rodrigues);

  /*! Apply a camera transform on the left. */
  void applyTransformAfter(const Pose3D& rhs_pose);
  void applyTransformAfter(const cv::Vec3f& translation, const cv::Vec3f& rotation_euler_angles);
  void applyTransformAfter(const cv::Vec3f& translation, const cv::Mat1d& rotation_matrix);
  void applyTransformAfterRodrigues(const cv::Vec3f& translation, const cv::Vec3f& rotation_rodrigues);

public:
  /*! Apply the camera transform on a given 3D point. */
  cv::Point3f cameraTransform(const cv::Point3f& p) const;

  /*! Apply the inverse camera transform on a given 3D point. */
  cv::Point3f invCameraTransform(const cv::Point3f& p) const;

  /*! Only apply the rotational part of the camera transform to a point. */
  cv::Point3f rotationTransform(const cv::Point3f& p) const;

  /*! Project a 3D point to image plane. */
  cv::Point3f projectToImage(const cv::Point3f& p) const;

  /*! Project a set of 3D points onto image plane. */
  void projectToImage(const cv::Mat4f& voxels, const cv::Mat1b& mask, cv::Mat4f& pixels) const;

  /*! Project a point from image plane to 3D using the given depth. */
  cv::Point3f unprojectFromImage(const cv::Point2f& p, double depth) const;
  cv::Point3f unprojectFromImage(const cv::Point3f& p) const
  { return unprojectFromImage(cv::Point2f(p.x,p.y), p.z); }

  /*! Project a set of image points to 3D. */
  void unprojectFromImage(const cv::Mat1f& pixels, const cv::Mat1b& mask, cv::Mat4f& voxels) const;

public:
  /*!
   * Compute the euclidian distance between two poses.
   * First component is translation, second is angle
   * between the two quaternions.
   */
  cv::Vec2f distanceWith(const Pose3D& rhs) const;

public:
  friend class Pose3DEigenAccessor;

private:
  double m_focal_x;
  double m_focal_y;
  double m_image_center_x;
  double m_image_center_y;
  bool m_has_camera_params;
  bool m_orthographic;
};

/*! Estimate a normal vector from neighborhood depth measurements. */
cv::Vec3f estimate_normal_from_depth(const cv::Mat1f& depth_yml,
                                     const Pose3D& pose,
                                     int r, int c,
                                     float depth_delta_limit = 0.03,
                                     const cv::Mat1f* dx = 0,
                                     const cv::Mat1f* dy = 0);

/*! Estimate the line of sight for the given pixel. */
cv::Vec3f camera_eye_vector(const Pose3D& pose, int r, int c);

/*!
 * Return the angle axis rotation (using Rodrigues representation) transforming
 * vector src into vector dst.
 */
cv::Vec3f compute_axis_angle_rotation(const cv::Vec3f& src, const cv::Vec3f& dst);

const NtkDebug& operator<<(const NtkDebug& os, const Pose3D& p);

} // ntk

#endif // NTK_GEOMETRY_POSE3D_H
