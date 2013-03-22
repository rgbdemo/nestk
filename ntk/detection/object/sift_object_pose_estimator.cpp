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

#include "sift_object_pose_estimator.h"
#include "located_feature.h"
#include "sift_point_match.h"
#include "sift_parameters.h"
#include "pose_2d.h"
#include "visual_object_view.h"
#include "object_detector.h"

#include <ntk/stats/moments.h>
#include <ntk/geometry/similarity_transform.h>
#include <ntk/geometry/polygon.h>

#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/numeric/cost_function.h>

#include <iterator>

using namespace ntk;
using namespace cv;

namespace ntk
{

static void removeMatchestoSamePoint(SiftPointMatchConstPtrSet& matches)
{
    bool has_changed = true;
    while (has_changed)
    {
        has_changed = false;
        SiftPointMatchConstPtrSet::iterator it, next_it, it2, next_it2;
        for (it = matches.begin(); it != matches.end(); it = next_it)
        {
            next_it = it; ++next_it;
            for (it2 = matches.begin(); it2 != matches.end(); it2 = next_it2)
            {
                next_it2 = it2; ++next_it2;
                if ((it != it2) &&
                        ((&((*it)->modelPoint()) == &((*it2)->modelPoint()))
                         || (&((*it)->obsPoint()) == &((*it2)->obsPoint()))))
                {
                    bool first_is_worse = false;
                    // first_is_worse = ((*it)->strength() < (*it2)->strength());
                    if (flt_eq((*it2)->geometricError(), (*it)->geometricError(), 1e-5))
                        first_is_worse = (*it)->strength() < (*it2)->strength();
                    else
                        first_is_worse = (*it)->geometricError() > (*it2)->geometricError();
                    if (first_is_worse)
                    {
                        matches.erase(it);
                        break;
                    }
                    else
                        matches.erase(it2);
                    has_changed = true;
                }
            }
            if (has_changed) break;
        }
    }
}

static void filterMatchesUsingDepth(SiftPointMatchConstPtrSet& matches, double depth_margin)
{
    std::vector<double> depth_values; depth_values.reserve(matches.size());
    foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
    {
        const LocatedFeature& p = (*it)->obsPoint();
        if (p.location().has_depth)
            depth_values.push_back(p.location().p_image.z);
    }
    if (depth_values.size() < 2) return;

    std::nth_element(depth_values.begin(), depth_values.begin()+depth_values.size()/2, depth_values.end());
    double median_depth = depth_values[depth_values.size()/2];

    SiftPointMatchConstPtrSet::const_iterator it, next_it;
    for (it = matches.begin(); it != matches.end(); it = next_it)
    {
        next_it = it; ++next_it;
        const LocatedFeature& p = (*it)->obsPoint();
        if (!p.location().has_depth)
            continue;
        double diff = p.location().p_image.z / median_depth;
        if (diff < (1-depth_margin) || diff > (1+depth_margin))
        {
            matches.erase(it);
        }
    }
}

static void filterMatchesOutside(SiftPointMatchConstPtrSet& matches, const ObjectPose& pose)
{
    SiftPointMatchConstPtrSet::const_iterator it, next_it;
    for (it = matches.begin(); it != matches.end(); it = next_it)
    {
        next_it = it; ++next_it;
        const LocatedFeature& p = (*it)->obsPoint();
        const ntk::Polygon2d& box = pose.projectedBoundingRect();
        cv::Rect_<float> r = bounding_box(box);
        // FIXME: temp
        if (!r.contains(Point2f(p.location().p_image.x, p.location().p_image.y)))
            matches.erase(it);
    }
}

struct reprojection_error_2d : public ntk::CostFunction
{
    reprojection_error_2d(const SimilarityTransform& initial_transform,
                          const SiftPointMatchConstPtrSet& matches)
        : CostFunction(4, matches.size()*2),
          initial_transform(initial_transform),
          matches(matches)
    {}

    virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
    {
        SimilarityTransform new_transform = initial_transform;
        new_transform.applyTransformAfter(Point2f(x[0],x[1]), x[2], x[3]);

        int i = 0;
        foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
        {
            const LocatedFeature& model_point = (*it)->modelPoint();
            const LocatedFeature& obs_point = (*it)->obsPoint();
            // double strength = (*it)->strength();
            double strength = 1;
            Point2f model_p (model_point.location().p_image.x, model_point.location().p_image.y);
            Point2f proj_p = new_transform.transform(model_p);
            fx[i*2] = (proj_p.x - obs_point.location().p_image.x) * strength;
            fx[i*2+1] = (proj_p.y - obs_point.location().p_image.y) * strength;
            ++i;
        }
    }

private:
    const SimilarityTransform& initial_transform;
    const SiftPointMatchConstPtrSet& matches;
};

struct avs_reprojection_error_3d : public ntk::CostFunction
{
    avs_reprojection_error_3d(const Pose3D& initial_pose,
                              const SiftPointMatchConstPtrSet& matches)
        : CostFunction(6, matches.size()*3),
          initial_pose(initial_pose),
          matches(matches)
    {}

    virtual void evaluate (const std::vector< double > &x, std::vector< double > &fx) const
    {
        const bool use_depth = true;

        // depth precision is about 3cm
        // spatial precision is about 1mm, hence this ratio
        const float depth_strength = 1;
        Pose3D new_pose = initial_pose;
        new_pose.applyTransformAfter(Vec3f(x[3],x[4],x[5]), cv::Vec3f(x[0],x[1],x[2]));
        int i = 0;
        std::fill(stl_bounds(fx), 0);
        foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
        {
            const LocatedFeature& model_point = (*it)->modelPoint();
            const LocatedFeature& obs_point = (*it)->obsPoint();
            if (model_point.location().has_depth)
            {
                // double strength = (*it)->strength();
                Point3f proj_p = new_pose.projectToImage(model_point.location().p_world);
                fx[i*3] = (proj_p.x - obs_point.location().p_image.x) / new_pose.meanFocal();
                fx[i*3+1] = (proj_p.y - obs_point.location().p_image.y) / new_pose.meanFocal();

                if (use_depth && obs_point.location().has_depth && model_point.location().has_depth)
                    fx[i*3+2] = (proj_p.z - obs_point.location().p_image.z) * depth_strength;
                else
                    // FIXME: quick hack to avoid putting 0 here and create a false error mean.
                    fx[i*3+2] = (fx[i*3]+fx[i*3+1])/2.0;
            }

            i = i + 1;
        }
    }

private:
    const Pose3D& initial_pose;
    const SiftPointMatchConstPtrSet& matches;
};

// atomic mean square pose estimation.
static double avs_rms_optimize_3d(Pose3D& pose3d,
                                  SiftPointMatchConstPtrSet& matches,
                                  std::vector<double>* p_fx = 0)
{
    std::vector<double> fx;
    std::vector<double> initial(6);
    avs_reprojection_error_3d f(pose3d, matches);
    LevenbergMarquartMinimizer optimizer;
    std::fill(stl_bounds(initial), 0);
    fx.resize(matches.size()*3 + 6);
    optimizer.minimize(f, initial);
    optimizer.diagnoseOutcome();
    f.evaluate(initial, fx);

    double error = f.outputNorm(initial);

    pose3d.applyTransformAfter(Vec3f(initial[3],initial[4],initial[5]), cv::Vec3f(initial[0], initial[1], initial[2]));

    if (p_fx)
        *p_fx = fx;
    return error;
}

static void compute_reprojection_error_2d_and_filter(SimilarityTransform& pose2d,
                                                     SiftPointMatchConstPtrSet& matches,
                                                     double error_threshold)
{
    SiftPointMatchConstPtrSet::const_iterator it, next_it;
    for(it = matches.begin(); it != matches.end(); it = next_it)
    {
        next_it = it; ++next_it;
        const LocatedFeature& model_point = (*it)->modelPoint();
        const LocatedFeature& obs_point = (*it)->obsPoint();
        Point2f model_p (model_point.location().p_image.x, model_point.location().p_image.y);
        Point2f proj_p = pose2d.transform(model_p);
        double e = ntk::math::sqr(proj_p.x - obs_point.location().p_image.x)
                + ntk::math::sqr(proj_p.y - obs_point.location().p_image.y);
        (*it)->setGeometricError(e);
        if (sqrt(e) > error_threshold)
        {
            matches.erase(it);
        }
    }
}

static void minimize_reprojection_error_2d(SimilarityTransform& pose2d,
                                           SiftPointMatchConstPtrSet& matches)
{
    std::vector<double> initial (4);
    std::vector<double> fx;

    ntk_dbg(2) << "====================";

    bool has_changed = false;
    do
    {
        reprojection_error_2d f(pose2d, matches);
        LevenbergMarquartMinimizer optimizer;

        std::fill(stl_bounds(initial), 0);

        fx.resize(matches.size()*2);
        f.evaluate(initial, fx);
        optimizer.minimize(f, initial);
        //optimizer.diagnoseOutcome();
        f.evaluate(initial, fx);

        for (int i = 0; i < fx.size(); ++i)
            fx[i] = ntk::math::abs(fx[i]);

        double mean=0, dev=0;
        ntk::values_moments(stl_bounds(fx), fx.size(), &mean, &dev);

        has_changed = false;
#if 0
        for (int i = 0; (matches.size() > 3) && i < matches.size()*2; i+=2)
        {
            if (((fx[i] > (mean+3.0*dev)) || (fx[i+1] > (mean+3.0*dev)))) // FIXME
            {
                ntk_dbg(2) << "Removing match " << i << " with err=" << fx[i] << ", " << fx[i+1];
                SiftPointMatchConstPtrSet::iterator it = matches.begin();
                std::advance(it, i/2);
                matches.erase(it);
                has_changed = true;
            }
        }
#endif
    } while (has_changed);

    ntk_assert(fx.size() == (matches.size()*2), "Inconsistency");

    {
        int i = 0;
        foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
        {
            (*it)->setGeometricError(fx[i] + fx[i+1]);
            i += 2;
        }
    }

    pose2d.applyTransformAfter(Point2f(initial[0], initial[1]), initial[2], initial[3]);
}

static void avs_rms_optimize_ransac(Pose3D& pose3d,
                                    SiftPointMatchConstPtrSet& matches)
{
    const double point_err_threshold = 0.05;
    const double initial_error_threshold = 0.05;
    const double max_final_error = 0.05; // 5 cm
    const int max_iterations = 20;
    const float min_consensus_support_percent = 0.2;
    const int min_support_points = std::max(5, (int)(matches.size()*min_consensus_support_percent));

    ntk_ensure(matches.size() > min_support_points, "Not enough points.");

    ntk_dbg(2) << "matches.size avant ransac:" << matches.size();

    std::vector<SiftPointMatchConstPtrSet::const_iterator> matches_vector(matches.size());
    { int i = 0;
        foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
                matches_vector[i++] = it;
    }

    cv::RNG rgen;
    double best_error = FLT_MAX;
    Pose3D best_pose = pose3d;
    Pose3D current_pose = pose3d;
    SiftPointMatchConstPtrSet current_matches;
    std::set<int> best_indices;
    std::set<int> indices;


    for (int it = 0; it < max_iterations && best_error > 0.01; ++it)
    {
        // initial set and model
        draw_k_different_numbers(rgen, indices, min_support_points, matches.size());

        current_pose = pose3d;
        current_matches.clear();
        foreach_const_it(it, indices, std::set<int>)
                current_matches.insert(*matches_vector[*it]);

        double initial_error = avs_rms_optimize_3d(current_pose, current_matches);
        initial_error /= matches.size();
        ntk_dbg_print(initial_error, 2);
        // Base solution not good enough.
        if (initial_error > initial_error_threshold)
            continue;

        indices.clear();

        // determine consensus set.
        foreach_idx(index, matches_vector)
        {          
            const SiftPointMatch& match = **matches_vector[index];
            const LocatedFeature& model_point = match.modelPoint();
            const LocatedFeature& obs_point = match.obsPoint();
            if (!model_point.location().has_depth)
                continue;

            Point3f proj_p = current_pose.projectToImage(model_point.location().p_world);
            double error = 0;
            error += ntk::math::sqr((proj_p.x - obs_point.location().p_image.x)/current_pose.meanFocal());
            error += ntk::math::sqr((proj_p.y - obs_point.location().p_image.y)/current_pose.meanFocal());
            if (obs_point.location().has_depth)
                error += ntk::math::sqr((proj_p.z - obs_point.location().p_image.z));
            error = sqrt(error);
            ntk_dbg_print(error, 2);

            if (error < point_err_threshold)
                indices.insert(index);
        }

        ntk_dbg_print(indices.size(), 2);
        ntk_dbg_print(min_support_points, 2);

        if (indices.size() < min_support_points)
            continue;

        ntk_dbg(2) << "Enough points";

        current_matches.clear();
        foreach_const_it(it, indices, std::set<int>)
        {
            current_matches.insert(*matches_vector[*it]);
        }
        // current_pose = pose3d;

        double iteration_error = avs_rms_optimize_3d(current_pose, current_matches);
        iteration_error /= matches.size();
        if (iteration_error > max_final_error)
            continue;

        // if (indices.size() > best_indices.size())
        if (iteration_error < best_error)
        {
            best_error = iteration_error;
            best_pose = current_pose;
            best_indices = indices;
        }
    } // end ransac loop    

    ntk_dbg_print(best_error, 1);

    if (best_error > max_final_error)
    {
        matches.clear();
        return;
    }

    current_matches.clear();
    foreach_const_it(it, best_indices, std::set<int>)
            current_matches.insert(*matches_vector[*it]);
    matches = current_matches;
    pose3d = best_pose;

    ntk_dbg_print(best_error, 1);

    ntk_dbg(2) << "matches.size after ransac:" << matches.size();
}

static void rms_optimize_iterative(Pose3D& pose3d,
                                   SiftPointMatchConstPtrSet& matches,
                                   double error_threshold)
{
    std::vector<double> fx;

    ntk_dbg(2) << "====================";

    bool has_changed = false;
    do
    {
        avs_rms_optimize_3d(pose3d, matches, &fx);

        for (int i = 0; i < fx.size(); ++i)
            fx[i] = ntk::math::abs(fx[i]);

        double mean=0, dev=0;
        ntk::values_moments(stl_bounds(fx), fx.size(), &mean, &dev);

        has_changed = false;
        for (int i = 0; (matches.size() > 3) && i < matches.size()*3; i+=3)
        {
            double e = sqrt(ntk::math::sqr(fx[i]) + ntk::math::sqr(fx[i+1]));
            if (e > error_threshold || fx[i] > mean+3.0*dev || fx[i+1] > mean+3.0*dev)
            {
                ntk_dbg(2) << "Removing match " << i << " with err=" << fx[i] << ", " << fx[i+1];
                SiftPointMatchConstPtrSet::iterator it = matches.begin();
                std::advance(it, i/3);
                matches.erase(it);
                has_changed = true;
            }
        }
    } while (has_changed && matches.size() > 5);

    {
        int i = 0;
        foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
        {
            (*it)->setGeometricError(fx[i] + fx[i+1]);
            i += 3;
        }
    }
}

static double compute_angle_mean(const std::vector<double>& angles)
{
    std::vector<double> cos_values (angles.size());
    std::vector<double> sin_values (angles.size());
    foreach_idx(i, angles)
    {
        cos_values[i] = cos(angles[i]);
        sin_values[i] = sin(angles[i]);
    }

    double cos_mean = 0;
    double sin_mean = 0;
    values_moments(stl_bounds(cos_values), cos_values.size(), &cos_mean, 0);
    values_moments(stl_bounds(sin_values), sin_values.size(), &sin_mean, 0);

    if (cos_mean < 0) return (atan(sin_mean/cos_mean) + M_PI);
    if (sin_mean < 0) return (atan(sin_mean/cos_mean) + 2.0*M_PI);
    return atan(sin_mean/cos_mean);
}

static void compute_mean_similarity_transform(SimilarityTransform& transform,
                                              SiftPointMatchConstPtrSet& matches)
{
    double angle_mean = 0;
    Point2f translation_mean (0,0);
    double scale_mean = 0;
    std::vector<double> angle_values (matches.size());
    int i = 0;
    foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
    {
        const SiftPointTransform& transform = (*it)->pointTransform();
        scale_mean += transform.scale;
        translation_mean += transform.translation;
        angle_values[i] = transform.orientation;
        ++i;
    }
    angle_mean = compute_angle_mean(angle_values);
    translation_mean *= (1.0/matches.size());
    scale_mean /= matches.size();

    transform = SimilarityTransform(translation_mean, angle_mean, scale_mean);
}

bool SiftObjectPoseEstimator :: computeFastPose(ObjectPose& pose,
                                                SiftPointMatchConstPtrSet& matches,
                                                const ObjectDetectorData& finder_data)
{
    const VisualObjectView& model = pose.modelView();
    SimilarityTransform similarity_transform;

    filterMatchesUsingDepth(matches, m_params.depth_margin);

    if (matches.size() < 1)
        return true;

    compute_mean_similarity_transform(similarity_transform, matches);
    ntk::Polygon2d polygon;
    similarity_transform.transform(pose.modelView().boundingRect(), polygon);
    Rect_<float> hull = polygon.boundingBox();

    removeMatchestoSamePoint(matches);

    if (matches.size() < 1)
        return true;

    compute_mean_similarity_transform(similarity_transform, matches);

    ntk::AffineTransform global_transform =
            ntk::AffineTransform()
            .rotated(similarity_transform.rotation())
            .scaled(similarity_transform.scale(),similarity_transform.scale())
            .translated(similarity_transform.translation()[0],
                        similarity_transform.translation()[1]);

    Pose2D pose_2d (global_transform);
    pose_2d.setSimilarityTransform(similarity_transform);
    pose.setPose2D(pose_2d);
    pose.finalize();
    return true;
}

bool SiftObjectPoseEstimator :: optimize(ObjectPose& pose,
                                         SiftPointMatchConstPtrSet& matches,
                                         const ObjectDetectorData& finder_data)
{
    TimeCount tc_optimize("optimize", 1);
    const VisualObjectView& model = pose.modelView();
    SimilarityTransform similarity_transform;

    filterMatchesUsingDepth(matches, m_params.depth_margin);

    if (matches.size() < 1)
        return false;

    compute_mean_similarity_transform(similarity_transform, matches);
    ntk::Polygon2d polygon;
    similarity_transform.transform(pose.modelView().boundingRect(), polygon);
    Rect_<float> hull = polygon.boundingBox();

    double max_projected_dim = ntk::math::max(hull.width, hull.height);
#if 1
    compute_reprojection_error_2d_and_filter(similarity_transform,
                                             matches,
                                             max_projected_dim*m_params.max_reprojection_error_percent);
#endif

    removeMatchestoSamePoint(matches);

    if (matches.size() < 1)
        return false;

    compute_mean_similarity_transform(similarity_transform, matches);

    ntk::AffineTransform global_transform =
            ntk::AffineTransform()
            .rotated(similarity_transform.rotation())
            .scaled(similarity_transform.scale(),similarity_transform.scale())
            .translated(similarity_transform.translation()[0],
                        similarity_transform.translation()[1]);

    Pose2D pose_2d (global_transform);
    pose_2d.setSimilarityTransform(similarity_transform);
    pose.setPose2D(pose_2d);
    pose.finalize();
#if 0
    filterMatchesOutside(matches, pose);
#endif

    ntk_dbg(1) << "after filtermatchesoutside";
    ntk_dbg_print(matches.size(), 1);

    if (!model.object().has3DModel())
        return true;

    if (!finder_data.image.calibration())
        return true;

    Pose3D pose_3d = model.objectPose();    
    pose_3d.setCameraParametersFromOpencv(finder_data.image.calibration()->rgb_intrinsics);

    double scale = 1.0/similarity_transform.scale();
    double theta = similarity_transform.rotation();
    cv::Vec3f euler_angles (0,0,-theta);
    //vgl_rotation_3d<double> r (0,0,0);

    pose_3d.applyTransformAfter(cv::Vec3f(0, 0, 0), euler_angles);

    cv::Point3f p;
    cv::Point2f projected_loc;
    SiftPointMatchConstPtrSet matches_with_depth;
    foreach_const_it(it, matches, SiftPointMatchConstPtrSet)
    {
        if ((*it)->modelPoint().location().has_depth)
            matches_with_depth.insert(*it);
    }

    ntk_dbg(1) << "after filtermatchesoutside";
    ntk_dbg_print(matches_with_depth.size(), 1);

    double old_z = 0, new_z = 0;
    cv::Point3f p0;
    if (matches_with_depth.size() > 0)
    {
        const SiftPointMatch* first_match_with_depth = *matches_with_depth.rbegin();
        p = first_match_with_depth->modelPoint().location().p_world;
        projected_loc = first_match_with_depth->obsPoint().imagePos();
        p0 = pose_3d.projectToImage(p);
        old_z = p0.z;
        if (first_match_with_depth->obsPoint().location().has_depth)
            new_z = first_match_with_depth->obsPoint().location().p_image.z;
        else
            new_z = old_z * scale;
    }
    else
    {
        p = cv::Point3f(0,0,-1);
        pose.computeProjections();
        projected_loc = pose.projectedCenter();
        p0 = pose_3d.projectToImage(p);
        old_z = p0.z;
        new_z = old_z * scale;
    }

    pose_3d.applyTransformAfter(cv::Vec3f(0, 0, old_z-new_z), cv::Vec3f(0,0,0));

    cv::Point3f p1 = pose_3d.projectToImage(p);

    double dx = projected_loc.x - p1.x;
    double dy = projected_loc.y - p1.y;

    double dx_3d = p1.z * dx / pose_3d.meanFocal();
    double dy_3d = p1.z * -dy / pose_3d.meanFocal();

    pose_3d.applyTransformAfter(cv::Vec3f(dx_3d, dy_3d, 0), cv::Vec3f(0,0,0));

    tc_optimize.elapsedMsecs(" -- 2d part: ");

    if (matches_with_depth.size() > 6)
    {
        avs_rms_optimize_ransac(pose_3d, matches_with_depth);
        ntk_dbg(1) << "after ransac";
        ntk_dbg_print(matches_with_depth.size(), 1);
    }
    else if (matches_with_depth.size() > 4)
    {
        rms_optimize_iterative(pose_3d, matches_with_depth,
                               max_projected_dim*m_params.max_reprojection_error_percent);
    }

    tc_optimize.elapsedMsecs(" -- 3d refinement: ");
    matches = matches_with_depth;

    pose.setPose3D(pose_3d);  
    pose.finalize();
    filterMatchesOutside(matches, pose);
    return true;
}

} // avs
