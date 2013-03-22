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


#include "relative_pose_estimator.h"

#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>
#include <ntk/stats/histogram.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/mesh/mesh.h>

// FIXME: disabled because of conflict between opencv flann and pcl flann.
// using namespace cv;
using cv::Vec3f;
using cv::Point3f;

namespace ntk
{

/*!
 * Compute the projection error of a 3D point cloud on a new image.
 * If feature points on the new image have depth information, it
 * will be taken into account.
 */
struct reprojection_error_3d : public ntk::CostFunction
{
    reprojection_error_3d(const Pose3D& initial_pose,
                          const std::vector<Point3f>& ref_points,
                          const std::vector<Point3f>& img_points,
                          bool use_depth)
        : CostFunction(6, ref_points.size()*(use_depth ? 3 : 2)),
          initial_pose(initial_pose),
          ref_points(ref_points),
          img_points(img_points),
          stride (use_depth ? 3 : 2),
          use_depth (use_depth)
    {
        ntk_assert(ref_points.size() == img_points.size(), "Invalid matches");
    }

    virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
    {
        Pose3D new_pose = initial_pose;
        new_pose.applyTransformAfter(Vec3f(x[3],x[4],x[5]), cv::Vec3f(x[0],x[1],x[2]));
        int err_i = 0;
        std::fill(stl_bounds(fx), 0);
        foreach_idx(p_i, ref_points)
        {
            const Point3f& ref_point = ref_points[p_i];
            const Point3f& img_point = img_points[p_i];
            Point3f proj_p = new_pose.projectToImage(ref_point);

            bool has_depth_i = use_depth && img_point.z > 1e-5;
            float norm_factor = 1.0;
            // this point has no depth, compensate the missing error value.
            if (use_depth && !has_depth_i) norm_factor = 3.0 / 2.0;

            fx[err_i*stride] = norm_factor * (proj_p.x - img_point.x) / new_pose.meanFocal();
            fx[err_i*stride+1] = norm_factor * (proj_p.y - img_point.y) / new_pose.meanFocal();

            if (use_depth && has_depth_i)
                fx[err_i*stride+2] = norm_factor * (proj_p.z - img_point.z);

            err_i = err_i + 1;
        }
    }

private:
    const Pose3D& initial_pose;
    const std::vector<Point3f>& ref_points;
    const std::vector<Point3f>& img_points;
    int stride;
    bool use_depth;
};

// atomic mean square pose estimation.
double rms_optimize_3d(Pose3D& pose3d,
                       const std::vector<Point3f>& ref_points,
                       const std::vector<Point3f>& img_points,
                       bool use_depth)
{
    std::vector<double> fx;
    std::vector<double> initial(6);
    reprojection_error_3d f(pose3d, ref_points, img_points, use_depth);
    LevenbergMarquartMinimizer optimizer;
    std::fill(stl_bounds(initial), 0);
    fx.resize(ref_points.size()*3);
    optimizer.minimize(f, initial);
    optimizer.diagnoseOutcome();
    f.evaluate(initial, fx);

    // FIXME: use normalized norm?
    double error = f.outputNorm(initial);

    pose3d.applyTransformAfter(Vec3f(initial[3],initial[4],initial[5]), cv::Vec3f(initial[0], initial[1], initial[2]));
    return error;
}

double rms_optimize_ransac(Pose3D& pose3d,
                           const std::vector<Point3f>& ref_points,
                           const std::vector<Point3f>& img_points,
                           std::vector<bool>& valid_points,
                           bool use_depth)
{
    // One centimeter. 1000*1000 comes from the error scale factor
    // in rms_optimize.
    const double rms_err_threshold = 0.005;
    const double compat_err_threshold = 0.05;
    const int max_iterations = 50;
    const int min_support_points = std::max(7, int(ref_points.size()/20));
    const float min_consensus_support_percent = 0.05f;

    ntk_assert(ref_points.size() > min_support_points, "Not enough points.");

    cv::RNG rgen;
    double best_error = FLT_MAX;
    Pose3D best_pose = pose3d;
    Pose3D current_pose = pose3d;
    std::vector<Point3f> current_ref_points;
    std::vector<Point3f> current_img_points;
    std::set<int> best_indices;
    std::set<int> indices;

    ntk_dbg_print(min_support_points, 2);

    for (int it = 0; it < max_iterations && best_error > 0.01; ++it)
    {
        // initial set and model
        draw_k_different_numbers(rgen, indices, min_support_points, ref_points.size());

        current_pose = pose3d;
        current_ref_points.clear();
        current_img_points.clear();
        current_ref_points.reserve(indices.size());
        current_img_points.reserve(indices.size());
        foreach_const_it(it, indices, std::set<int>)
        {
            current_ref_points.push_back(ref_points[*it]);
            current_img_points.push_back(img_points[*it]);
        }

        double mean_initial_error = rms_optimize_3d(current_pose, current_ref_points, current_img_points, use_depth);
        mean_initial_error /= current_ref_points.size();

        ntk_dbg_print(mean_initial_error, 2);

        // Base solution not good enough.
        if (mean_initial_error > rms_err_threshold)
            continue;

        ntk_dbg_print(mean_initial_error, 2);
        // determine consensus set.
        foreach_idx(index, ref_points)
        {
            if (indices.find(index) != indices.end())
                continue;

            Point3f proj_p = current_pose.projectToImage(ref_points[index]);
            double error = 0;
            error += ntk::math::sqr(proj_p.x - img_points[index].x) / current_pose.meanFocal();
            error += ntk::math::sqr(proj_p.y - img_points[index].y) / current_pose.meanFocal();
            if (img_points[index].z > 1e-5)
                error += ntk::math::sqr((proj_p.z - img_points[index].z));
            error = sqrt(error);

            if (error < compat_err_threshold)
                indices.insert(index);
        }

        ntk_dbg_print(indices.size(), 2);
        ntk_dbg_print(min_consensus_support_percent*ref_points.size(), 2);
        if (indices.size() < (min_consensus_support_percent*ref_points.size()))
            continue;

        current_ref_points.clear();
        current_img_points.clear();
        foreach_const_it(it, indices, std::set<int>)
        {
            current_ref_points.push_back(ref_points[*it]);
            current_img_points.push_back(img_points[*it]);
        }
        // current_pose = pose3d;

        double iteration_error = rms_optimize_3d(current_pose, current_ref_points, current_img_points, use_depth);
        // iteration_error /= current_ref_points.size();
        ntk_dbg_print(iteration_error, 2);
        if (iteration_error < best_error)
        {
            best_error = iteration_error;
            best_pose = current_pose;
            best_indices = indices;
        }
    } // end ransac loop

    valid_points.resize(ref_points.size(), false);
    foreach_const_it(it, best_indices, std::set<int>)
            valid_points[*it] = true;
    ntk_dbg_print(best_indices.size(), 2);
    pose3d = best_pose;
    return best_error;
}

/*!
 * Compute the projection error of a 3D point cloud on a new image.
 * If feature points on the new image have depth information, it
 * will be taken into account.
 */
struct reprojection_error_depth : public ntk::CostFunction
{
    reprojection_error_depth(const Pose3D& initial_pose,
                          const std::vector<Point3f>& ref_points,
                          const cv::Mat1f& depth_im,
                          float max_distance)
        : CostFunction(6, ref_points.size()),
          initial_pose(initial_pose),
          ref_points(ref_points),
          depth_im(depth_im),
          max_distance(max_distance)
    {

    }

    virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
    {
        const bool use_depth = true;
        Pose3D new_pose = initial_pose;
        new_pose.applyTransformAfter(Vec3f(x[3],x[4],x[5]), cv::Vec3f(x[0],x[1],x[2]));
        int err_i = 0;
        std::fill(stl_bounds(fx), 0);
        foreach_idx(p_i, ref_points)
        {
            const Point3f& ref_point = ref_points[p_i];
            Point3f proj_p = new_pose.projectToImage(ref_point);
            if (!is_yx_in_range(depth_im, proj_p.y, proj_p.x))
                continue;

            float d = depth_im(proj_p.y, proj_p.x);
            if (d < 1e-5)
                continue;

            float diff = proj_p.z - d;
            if (std::abs(diff) > max_distance)
                diff = max_distance;

            fx[err_i] = diff;
            err_i = err_i + 1;
        }
    }

private:
    const Pose3D& initial_pose;
    const std::vector<Point3f>& ref_points;
    const cv::Mat1f& depth_im;
    float max_distance;
};

double rms_optimize_against_depth_image(Pose3D& pose3d,
                                        const std::vector<Point3f>& ref_points,
                                        const cv::Mat1f& depth_im,
                                        float max_distance)
{
    std::vector<double> fx;
    std::vector<double> initial(6);
    reprojection_error_depth f(pose3d, ref_points, depth_im, max_distance);
    LevenbergMarquartMinimizer optimizer;
    std::fill(stl_bounds(initial), 0);
    fx.resize(ref_points.size());
    optimizer.minimize(f, initial);
    optimizer.diagnoseOutcome();
    f.evaluate(initial, fx);

    // FIXME: use normalized norm?
    double error = f.outputNorm(initial);

    pose3d.applyTransformAfter(Vec3f(initial[3],initial[4],initial[5]), cv::Vec3f(initial[0], initial[1], initial[2]));
    return error;
}

} // ntk
