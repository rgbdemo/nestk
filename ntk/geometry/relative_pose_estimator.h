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

#ifndef NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H
#define NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H

#include <ntk/core.h>
#include <ntk/camera/rgbd_image.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/camera/rgbd_calibration.h>

#if defined(NESTK_USE_PCL) || defined(USE_PCL)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ntk/mesh/pcl_utils.h>
#endif

namespace ntk
{

class PoseEstimator
{
public:
    //! Estimate the pose for the updated data.
    //! \return true is pose successfully estimated, false otherwise.
    virtual bool estimateNewPose() = 0;

    const Pose3D& estimatedPose() const { return m_estimated_pose; }
    Pose3D& estimatedPose() { return m_estimated_pose; }

protected:
    Pose3D m_estimated_pose;
};

/*!
 * Estimate the 3D pose of a source image or point cloud by estimating the delta pose
 * w.r.t. a target reference.
 */
class RelativePoseEstimator
{
public:
    virtual ~RelativePoseEstimator() {}

    //! Estimate the relative pose for the updated data.
    //! \return true is pose successfully estimated, false otherwise.
    virtual bool estimateNewPose() = 0;

    //! Return last estimated pose.
    const Pose3D& estimatedSourcePose() const { return m_estimated_pose; }
    Pose3D& estimatedSourcePose() { return m_estimated_pose; }

    void setInitialSourcePoseEstimate(const Pose3D& pose) { m_initial_pose = pose; m_estimated_pose = pose; }
    void setTargetPose(const Pose3D& pose) { m_target_pose = pose; }

protected:
    Pose3D m_initial_pose;
    Pose3D m_target_pose;
    Pose3D m_estimated_pose;
};

/*!
 * Compute relative pose information betwen two RGBDImages.
 */
class RelativePoseEstimatorFromImages : public RelativePoseEstimator
{
public:
    RelativePoseEstimatorFromImages()
        : m_source_image(0),
          m_target_image(0)
    {}

    virtual void setSourceImage(const RGBDImage& image)
    { m_source_image = &image; }

    virtual void setTargetImage(const RGBDImage& image)
    { m_target_image = &image; m_estimated_pose.setCameraParametersFromOpencv(image.calibration()->depth_intrinsics); }

protected:
    const RGBDImage* m_source_image;
    const RGBDImage* m_target_image;
};

#if defined(NESTK_USE_PCL) || defined(USE_PCL)
/*!
 * Compute relative pose information betwen PCL point clouds.
 */
template <class PointT>
class RelativePoseEstimatorFromPointClouds : public RelativePoseEstimatorFromImages
{
public:
    typedef pcl::PointCloud<PointT> PointCloudType;
    typedef typename PointCloudType::Ptr PointCloudPtr;
    typedef typename PointCloudType::ConstPtr PointCloudConstPtr;

public:
    RelativePoseEstimatorFromPointClouds()
    {}

    virtual void setSourceImage(const RGBDImage& image)
    {
        PointCloudPtr cloud(new PointCloudType);
        rgbdImageToPointCloud(*cloud, image);
        m_source_cloud = cloud; // FIXME: Find out whether the removal of the (deep-copying) cloud.makeShared() call sped things up.
    }

    virtual void setTargetImage(const RGBDImage& image)
    {
        PointCloudPtr cloud(new PointCloudType);
        rgbdImageToPointCloud(*cloud, image);
        m_target_cloud = cloud; // FIXME: Find out whether the removal of the (deep-copying) cloud.makeShared() call sped things up.
    }

    virtual void setSourceCloud(PointCloudConstPtr cloud)
    { m_source_cloud = cloud; }

    virtual void setTargetCloud(PointCloudConstPtr cloud)
    { m_target_cloud = cloud; }

protected:
    PointCloudConstPtr m_source_cloud;
    PointCloudConstPtr m_target_cloud;
};
#endif

double rms_optimize_ransac(Pose3D& pose3d,
                           const std::vector<cv::Point3f>& ref_points,
                           const std::vector<cv::Point3f>& img_points,
                           std::vector<bool>& valid_points,
                           bool use_depth);

double rms_optimize_3d(Pose3D& pose3d,
                       const std::vector<cv::Point3f>& ref_points,
                       const std::vector<cv::Point3f>& img_points,
                       bool use_depth);

double rms_optimize_against_depth_image(Pose3D& pose3d,
                                        const std::vector<cv::Point3f>& ref_points,
                                        const cv::Mat1f& depth_im,
                                        float max_distance);

} // ntk

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_H
