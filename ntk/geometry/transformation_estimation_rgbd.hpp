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


#ifndef NESTK_GEOMETRY_TRANSFORMATION_ESTIMATOR_RGBD_HPP
# define NESTK_GEOMETRY_TRANSFORMATION_ESTIMATOR_RGBD_HPP

#include <pcl/common/eigen.h>
#include <ntk/numeric/cost_function.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/debug.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/eigen_utils.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef HAVE_PCL_GREATER_THAN_1_6_0

using namespace pcl;

namespace ntk
{

template <class PointSource, class PointTarget, class Scalar>
struct TransformRGBDCostFunction : public CostFunction
{
    typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
    typedef Eigen::Transform<Scalar,3,Eigen::Isometry> Isometry3;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;

    TransformRGBDCostFunction(pcl::ConstCloudIterator<PointSource>& source_it,
                              pcl::ConstCloudIterator<PointTarget>& target_it,
                              const std::vector<cv::Point3f>* target_points_3d,
                              const std::vector<cv::Point3f>* source_image_points,
                              const ntk::Pose3D& source_rgb_pose,
                              const Isometry3& accumulated_transform)
        : CostFunction(6, source_it.size() + (target_points_3d ? 2*target_points_3d->size() : 0)),
          source_it(source_it),
          target_it(target_it),
          target_points_3d(target_points_3d),
          source_image_points(source_image_points),
          source_rgb_pose(source_rgb_pose),
          accumulated_transform(accumulated_transform)
    {}

    Isometry3 getEigenTransform(const std::vector<double>& x) const
    {
        Vector3 axis (x[0], x[1], x[2]);
        Scalar norm = axis.norm();
        axis.normalize();

        Isometry3 transform;
        transform.setIdentity();
        if (norm > 1e-10)
            transform.prerotate(Eigen::AngleAxis<Scalar>(norm, axis));
        transform.pretranslate(Vector3(x[3], x[4], x[5]));
        return transform;
    }

    virtual void evaluate(const std::vector<double>& x, std::vector<double>& fx) const
    {
        Isometry3 transform = getEigenTransform(x);
        Isometry3 rgb_transform = (transform * accumulated_transform).inverse();
        // Isometry3 rgb_transform = transform*accumulated_transform;

        std::fill(stl_bounds(fx), 0.0);
        float mean_focal = source_rgb_pose.isValid() ? source_rgb_pose.meanFocal() : 1000.f;

        const int nb_rgb_points = (target_points_3d ? target_points_3d->size() : 0);
        const float rgb_weight = source_it.size() > 0 ? 0.1f * source_it.size() : 1.0f;
        const float icp_weight = nb_rgb_points > 0 ? nb_rgb_points : 1.0f;
        for (int i = 0; i < nb_rgb_points; ++i)
        {
            Vector3 p3d; toEigen((*target_points_3d)[i], p3d);
            // ntk_dbg_print(cv::Vec3f(p3d(0),p3d(1),p3d(2)), 1);
            p3d = rgb_transform * p3d;
            cv::Point3f proj = source_rgb_pose.projectToImage(toVec3f(p3d));
            const cv::Point3f& target_p = (*source_image_points)[i];
            float dx = (proj.x - target_p.x);
            float dy = (proj.y - target_p.y);
            // Non-linear error term. If it is within the uncertainty range, leave ICP alone drive the process.
            fx[i*2] = std::abs(dx) > 2.f ? rgb_weight * dx : 0.f;
            fx[i*2+1] = std::abs(dy) > 2.f ? rgb_weight * dy : 0.f;
            // ntk_dbg_print(cv::Vec3f(p3d(0),p3d(1),p3d(2)), 1);
            // ntk_dbg_print(proj, 1);
            // ntk_dbg_print(target_p, 1);
            // ntk_dbg_print(fx[i*2], 1);
        }
        const int offset = nb_rgb_points*2;

        source_it.reset();
        target_it.reset();
        const int nb_points_3d = source_it.size();
        for (int i = 0; i < nb_points_3d; ++i)
        {
            Vector3 s = source_it->getVector3fMap();
            s = transform * s;
            Vector3 t = target_it->getVector3fMap();
            Vector3 n = target_it->getNormalVector3fMap();
            // Vector3 diff = (s-t); // Point to point
            Scalar diff = (s-t).dot(n); // Point to plane

            fx[offset+i] = icp_weight * mean_focal * diff;
            // fx[offset + i*3] = diff(0);
            // fx[offset + i*3+1] = diff(1);
            // fx[offset + i*3+2] = diff(2);
            ++source_it;
            ++target_it;
        }
    }

    pcl::ConstCloudIterator<PointSource>& source_it;
    pcl::ConstCloudIterator<PointTarget>& target_it;
    const std::vector<cv::Point3f>* target_points_3d;
    const std::vector<cv::Point3f>* source_image_points;
    const ntk::Pose3D& source_rgb_pose;
    Isometry3 accumulated_transform;
};

} // ntk

namespace ntk
{

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
TransformationEstimationRGBD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix,
    const std::vector<Scalar> &) const
{
  size_t nr_points = cloud_src.points.size ();
  if (cloud_tgt.points.size () != nr_points)
  {
    PCL_ERROR ("[pcl::TransformationEstimationRGBD::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", nr_points, cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
TransformationEstimationRGBD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix,
    const std::vector<Scalar> &) const
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::TransformationSVD::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
TransformationEstimationRGBD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Matrix4 &transformation_matrix,
    const std::vector<Scalar> &) const
{
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::TransformationEstimationRGBD::estimateRigidTransformation] Number or points in source (%zu) differs than target (%zu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  ConstCloudIterator<PointSource> source_it (cloud_src, indices_src);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, indices_tgt);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> void
TransformationEstimationRGBD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Matrix4 &transformation_matrix) const
{
  ConstCloudIterator<PointSource> source_it (cloud_src, correspondences, true);
  ConstCloudIterator<PointTarget> target_it (cloud_tgt, correspondences, false);
  estimateRigidTransformation (source_it, target_it, transformation_matrix);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
TransformationEstimationRGBD<PointSource, PointTarget, Scalar>::estimateRigidTransformation (
    ConstCloudIterator<PointSource>& source_it,
    ConstCloudIterator<PointTarget>& target_it,
    Matrix4 &transformation_matrix) const
{
    typedef Eigen::Transform<Scalar,3,Eigen::Isometry> Isometry3;
    std::vector<double> fx;
    std::vector<double> initial(6);
    TransformRGBDCostFunction<PointSource,PointTarget,Scalar> f(source_it,
                                                                target_it,
                                                                target_points_3d,
                                                                source_image_points,
                                                                source_rgb_pose,
                                                                Isometry3(registration_base->getFinalTransformation()));
    LevenbergMarquartMinimizer optimizer;
    std::fill(stl_bounds(initial), 0);
    const int error_size = source_it.size() + (target_points_3d ? 2*target_points_3d->size() : 0);
    fx.resize(error_size);
    optimizer.minimize(f, initial);
    optimizer.diagnoseOutcome(1);

    // FIXME: use normalized norm?
    double error = f.outputNorm(initial);
    ntk_dbg_print(error, 1);
    transformation_matrix = f.getEigenTransform(initial).matrix();
}

} // ntk

#endif

#endif // NESTK_GEOMETRY_TRANSFORMATION_ESTIMATOR_RGBD_HPP
