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


#include "incremental_pose_estimator_from_rgb_features.h"

#include <ntk/geometry/relative_pose_estimator_from_image.h>
#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>
#include <ntk/stats/histogram.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/mesh/mesh.h>
#include <ntk/hub.h>

// FIXME: disable until PCL and OpenCV conflict get solved.
#undef NESTK_USE_PCL
#undef USE_PCL

#if defined(NESTK_USE_PCL) || defined(USE_PCL)
# include <pcl/registration/icp.h>
# include <ntk/mesh/pcl_utils.h>
# include <pcl/filters/voxel_grid.h>
# include <ntk/mesh/pcl_utils.h>
using namespace pcl;
#endif

// FIXME: disabled because of conflict between opencv flann and pcl flann.
// using namespace cv;
using cv::Vec3f;
using cv::Point3f;

namespace ntk
{

/*!
 * Compute the number of closest feature matches for each previous view.
 */
int IncrementalPoseEstimatorFromRgbFeatures::
computeNumMatchesWithPrevious(const RGBDImage& image,
                              const FeatureSet& features,
                              std::vector<cv::DMatch>& best_matches)
{
    const int min_number_of_matches = 100;
    int best_prev_image = 0;
    // If at least 100 matches have been found with one image, stop searching.
    // Start with the last one, more likely to have a similar point of view.

    for (int i = m_features.size()-1;
         i >= 0 && best_matches.size() < min_number_of_matches;
         --i)
    {
        std::vector<cv::DMatch> current_matches;
        m_features[i]->matchWith(features, current_matches, 0.8f*0.8f);
        ntk_dbg_print(current_matches.size(), 1);
        if (current_matches.size() > best_matches.size())
        {
            best_prev_image = i;
            best_matches = current_matches;
        }
    }
    return best_prev_image;
}

bool IncrementalPoseEstimatorFromRgbFeatures::
estimateDeltaPose(Pose3D& new_depth_pose,
                  const RGBDImage& image,
                  ntk::Ptr<FeatureSet> image_features,
                  const std::vector<cv::DMatch>& best_matches,
                  int closest_view_index)
{
    const float err_threshold = 0.005f;

    ntk_dbg_print(new_depth_pose, 2);
    const ImageData& ref_image_data = m_image_data[closest_view_index];

    RelativePoseEstimatorFromRgbFeatures estimator (m_feature_parameters);
    estimator.setTargetPose(new_depth_pose);
    estimator.setTargetImage(ref_image_data.image, m_features[closest_view_index]);
    estimator.setSourceImage(image, image_features);
    bool ok = estimator.estimateNewPose();
    if (!ok)
        return false;
    new_depth_pose = estimator.estimatedSourcePose();
    return ok;
}

#if 0
bool IncrementalPoseEstimatorFromRgbFeatures::
estimateDeltaPose(Pose3D& new_rgb_pose,
                  const RGBDImage& image,
                  const FeatureSet& image_features,
                  const std::vector<cv::DMatch>& best_matches,
                  std::vector<bool>& valid_matches,
                  int closest_view_index)
{
    const float err_threshold = 0.005f;

    ntk_dbg_print(new_rgb_pose, 2);
    const ImageData& ref_image_data = m_image_data[closest_view_index];

    ntk_dbg_print(best_matches.size(), 2);
    if (best_matches.size() < 10)
    {
        ntk_dbg(1) << "Not enough point matches (< 10)";
        return false;
    }

    std::vector<Point3f> ref_points;
    std::vector<Point3f> img_points;

    foreach_idx(i, best_matches)
    {
        const cv::DMatch& m = best_matches[i];
        const FeatureSet& ref_set = *m_features[closest_view_index];
        const FeaturePoint& ref_loc = ref_set.locations()[m.trainIdx];
        const FeaturePoint& img_loc = image_features.locations()[m.queryIdx];

        ntk_assert(ref_loc.depth > 0, "Match without depth, should not appear");

        cv::Point3f img3d (img_loc.pt.x,
                           img_loc.pt.y,
                           img_loc.depth);

        ref_points.push_back(ref_loc.p3d);
        img_points.push_back(img3d);
    }

    ntk_dbg_print(ref_points.size(), 2);
    if (ref_points.size() < 10)
    {
        ntk_dbg(2) << "Not enough matches with depth";
        return false;
    }

    // double error = rms_optimize_3d(new_pose, ref_points, img_points);
    double error = rms_optimize_ransac(new_rgb_pose, ref_points, img_points, valid_matches, false /* use_depth */);
    error /= ref_points.size();

    ntk_dbg_print(error, 1);
    ntk_dbg_print(new_rgb_pose, 2);

    if (error < err_threshold)
        return true;
    else
        return false;
}
#endif

static void copy_only_useful_fields(const RGBDImage& src, RGBDImage& other)
{
    src.rgb().copyTo(other.rgbRef());
    other.setCalibration(src.calibration());
    other.setDirectory(src.directory());
    other.setCameraSerial(src.cameraSerial());
    other.setTimestamp(src.timestamp());
}

bool IncrementalPoseEstimatorFromRgbFeatures::estimateCurrentPose()
{
    const double max_delta_rotation = ntk::deg_to_rad(45.0); // 45 degrees
    const double max_delta_translation = 0.4; // 40 cm
    const int icp_cloud_samples = 3000;

    ntk::TimeCount tc("RelativePoseEstimator", 1);

    RGBDImage& image = m_new_image;

    ntk_ensure(image.calibration(), "Image must be calibrated.");
    bool first_pass = false;
    if (!m_current_pose.isValid())
    {
        m_current_pose = *image.calibration()->depth_pose;
        first_pass = true;
    }

    ntk_ensure(image.mappedDepth().data, "Image must have depth mapping.");

    ntk::Ptr<FeatureSet> image_features (new FeatureSet);
    image_features->extractFromImage(image, m_feature_parameters);
    tc.elapsedMsecs(" -- extract features from Image -- ");

#if 0
    Pose3D new_pose = *image.calibration()->depth_pose;
    Pose3D new_rgb_pose = new_pose;
    new_rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                               image.calibration()->R, image.calibration()->T);
#else
    Pose3D new_pose = *image.calibration()->depth_pose;
#endif
    bool pose_ok = true;

    int closest_view_index = -1;

    if (m_image_data.size() > 0)
    {
        std::vector<cv::DMatch> best_matches;
        closest_view_index = computeNumMatchesWithPrevious(image, *image_features, best_matches);
        tc.elapsedMsecs(" -- computeNumMatchesWithPrevious -- ");
        ntk_dbg_print(closest_view_index, 1);
        ntk_dbg_print(best_matches.size(), 1);       

#ifdef HEAVY_DEBUG
        imwrite("/tmp/debug_matches.png", debug_img);
#endif

        new_pose = m_image_data[closest_view_index].depth_pose;

        if (best_matches.size() > 0)
        {
            Pose3D delta_pose = new_pose;

            if (!estimateDeltaPose(new_pose, image, image_features, best_matches, closest_view_index))
                pose_ok = false;

            // Compute the difference between the reference image pose and the new image one.
            delta_pose.invert();
            delta_pose.applyTransformBefore(new_pose);

            // If the delta pose is too big, discard the image. The camera cannot move
            // so fast and this usually result from bad feature matches.
            double delta_rotation = cv::norm(delta_pose.cvRodriguesRotation());
            double delta_translation = cv::norm(delta_pose.cvTranslation());
            ntk_dbg_print(delta_rotation, 2);
            ntk_dbg_print(delta_translation, 2);
            if (delta_rotation > max_delta_rotation || delta_translation > max_delta_translation)
                pose_ok = false;
        }
        else
        {
            pose_ok = false;
        }
    }

    // FIXME: just to see if ICP is working.
    // new_pose.applyTransformAfter(Vec3f(0.02,0.01,0.01), Vec3f(0.01,0.01,0.01));

#ifdef NESTK_USE_PCL
    if (pose_ok && m_use_icp)
    {
        TimeCount tc("ICP");
        pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud;
        sampled_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        sampledRgbdImageToPointCloud(*sampled_cloud, image, new_pose, icp_cloud_samples);
        tc.elapsedMsecs(" -- sample image -- ");
        if (!first_pass)
            pose_ok &= optimizeWithICP(sampled_cloud, new_pose, closest_view_index);
        tc.elapsedMsecs(" -- optimization -- ");
    }
#endif

    if (pose_ok)
    {
        // m_features[closest_view_index]->drawMatches(m_image_data[closest_view_index].color, image.rgb(), *image_features, best_matches, debug_img);
        if (closest_view_index >= 0)
        {
            m_features[closest_view_index]->draw(m_image_data[closest_view_index].image.rgb(), m_feedback_image);
            hub::setImage("features", m_feedback_image);
        }

        Pose3D new_rgb_pose = new_pose;
        new_rgb_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                                   image.calibration()->R, image.calibration()->T);


        m_current_pose = new_pose;
        if (m_incremental_model || first_pass)
        {
            ImageData image_data;
            copy_only_useful_fields(image, image_data.image);
#ifdef NESTK_USE_PCL
            if (m_use_icp)
            {
                image_data.sampled_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
                sampledRgbdImageToPointCloud(*image_data.sampled_cloud, image, new_pose, icp_cloud_samples);
            }
#endif
            image_data.depth_pose = new_pose;
            image_features->compute3dLocation(new_rgb_pose);
            m_features.push_back(image_features);
            ntk_dbg_print(image_features->locations().size(), 1);
            m_image_data.push_back(image_data);
            tc.elapsedMsecs(" -- Finishing -- ");
        }
        return true;
    }
    else
    {
        if (!m_feedback_image.empty())
        {
            for_all_rc (m_feedback_image)
            {
                m_feedback_image(r,c)[2] = ntk::math::max(m_feedback_image(r,c)[2], 255);
            }
            hub::setImage("features", m_feedback_image);
        }

        return false;
    }
}

void IncrementalPoseEstimatorFromRgbFeatures::reset()
{
    m_features.clear();
    m_image_data.clear();
    m_current_pose = Pose3D();
}

#if defined(NESTK_USE_PCL) || defined(USE_PCL)

bool
IncrementalPoseEstimatorFromRgbFeatures::optimizeWithICP(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_source,
    Pose3D& depth_pose,
    int closest_view_index)
{
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_target = m_image_data[closest_view_index].sampled_cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_reg;

    Pose3D new_depth_pose = depth_pose;

    ntk_dbg_print(cloud_source->points.size(), 1);
    ntk_dbg_print(cloud_target->points.size(), 1);

    IterativeClosestPoint<PointXYZ, PointXYZ> reg;

    reg.setInputCloud (cloud_target);
    reg.setInputTarget (cloud_source);

    reg.setMaximumIterations (50);
    reg.setTransformationEpsilon (1e-5);
    reg.setMaxCorrespondenceDistance (0.5);
    reg.align (cloud_reg);

    if (!reg.hasConverged())
    {
        ntk_dbg(1) << "ICP did not converge, ignoring.";
        return false;
    }

    ntk_dbg_print(reg.getFitnessScore(), 1);

    Eigen::Matrix4f t = reg.getFinalTransformation ();
    cv::Mat1f T(4,4);
    //toOpencv(t,T);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            T(r,c) = t(r,c);

    Pose3D icp_pose;
    icp_pose.setCameraTransform(T);

    new_depth_pose.applyTransformAfter(icp_pose);
    depth_pose = new_depth_pose;
    return true;
}
#endif // NESTK_USE_PCL

} // ntk
