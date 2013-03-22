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

#ifndef NESTK_GEOMETRY_TRANSFORMATION_ESTIMATION_RGBD_H
#define NESTK_GEOMETRY_TRANSFORMATION_ESTIMATION_RGBD_H

#include <ntk/core.h>
#include <ntk/geometry/pose_3d.h>

#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation.h>

#ifdef HAVE_PCL_GREATER_THAN_1_6_0

namespace ntk
{
/** @b TransformationEstimationRGBD implements SVD-based estimation of
      * the transformation aligning the given correspondences.
      *
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Dirk Holz, Radu B. Rusu
      * \ingroup registration
      */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class TransformationEstimationRGBD : public pcl::registration::TransformationEstimation<PointSource, PointTarget, Scalar>
{
public:
    typedef boost::shared_ptr<TransformationEstimationRGBD<PointSource, PointTarget, Scalar> > Ptr;
    typedef boost::shared_ptr<const TransformationEstimationRGBD<PointSource, PointTarget, Scalar> > ConstPtr;

    typedef typename pcl::registration::TransformationEstimation<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;

    TransformationEstimationRGBD (pcl::Registration<PointSource, PointTarget>* registration_base)
        : registration_base(registration_base),
          target_points_3d(0),
          source_image_points(0)
    {}

    virtual ~TransformationEstimationRGBD () {}

    /** \brief Set colors features. */
    void setColorFeatures(const Pose3D& source_rgb_pose,
                          const std::vector<cv::Point3f>& target_points_3d,
                          const std::vector<cv::Point3f>& source_image_points)
    {
        this->source_rgb_pose = source_rgb_pose;
        this->target_points_3d = &target_points_3d;
        this->source_image_points = &source_image_points;
    }

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          * \param[in] weights weights for the point correspondences - not used in this class
          */
    void
    estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix,
            const std::vector<Scalar> &weights = std::vector<Scalar> ()) const;

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          * \param[in] weights weights for the point correspondences - not used in this class
          */
    void
    estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix,
            const std::vector<Scalar> &weights = std::vector<Scalar> ()) const;

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interst points from \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          * \param[in] weights weights for the point correspondences - not used in this class
          */
    void
    estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Matrix4 &transformation_matrix,
            const std::vector<Scalar> &weights = std::vector<Scalar> ()) const;

    /** \brief Estimate a rigid rotation transformation between a source and a target point cloud using SVD.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
    void
    estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const pcl::Correspondences &correspondences,
            Matrix4 &transformation_matrix) const;

protected:

    /** \brief Estimate a rigid rotation transformation between a source and a target
          * \param[in] source_it an iterator over the source point cloud dataset
          * \param[in] target_it an iterator over the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
    void
    estimateRigidTransformation (pcl::ConstCloudIterator<PointSource>& source_it,
                                 pcl::ConstCloudIterator<PointTarget>& target_it,
                                 Matrix4 &transformation_matrix) const;

private:
    pcl::Registration<PointSource, PointTarget>* registration_base;
    const std::vector<cv::Point3f>* target_points_3d;
    const std::vector<cv::Point3f>* source_image_points;
    ntk::Pose3D source_rgb_pose;
};

} // ntk

#endif

#endif // NESTK_GEOMETRY_TRANSFORMATION_ESTIMATION_RGBD_H
