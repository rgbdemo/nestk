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

#include "relative_pose_estimator_from_image.h"

#include "relative_pose_estimator_rgbd_icp.h"

#include <ntk/camera/rgbd_processor.h>
#include <ntk/mesh/pcl_utils.h>
#include <ntk/mesh/mesh_generator.h>
#include <ntk/utils/time.h>

using cv::Vec3f;
using cv::Point3f;

// #define HEAVY_DEBUG

namespace ntk
{

bool RelativePoseEstimatorFromRgbFeatures::
estimateNewPose(Pose3D& new_pose,
                const RGBDImage& image,
                const FeatureSet& image_features,
                const std::vector<cv::DMatch>& matches)
{
    const float err_threshold = 0.005f;

    std::vector<Point3f> ref_points;
    std::vector<Point3f> img_points;
    std::vector<float> distances;
    foreach_idx(i, matches)
    {
        const cv::DMatch& m = matches[i];
        const FeaturePoint& ref_loc = m_target_features->locations()[m.trainIdx];
        const FeaturePoint& img_loc = image_features.locations()[m.queryIdx];

        ntk_assert(ref_loc.depth > 0, "Match without depth, should not appear");

        cv::Point3f img3d (img_loc.pt.x,
                           img_loc.pt.y,
                           img_loc.depth);

        ref_points.push_back(ref_loc.p3d);
        img_points.push_back(img3d);
        distances.push_back(m.distance);
    }

    ntk_dbg_print(ref_points.size(), 2);
    if (ref_points.size() < 10)
    {
        ntk_dbg(2) << "Not enough matches with depth";
        return false;
    }

    std::vector<bool> valid_points;
    double error = rms_optimize_ransac(new_pose, ref_points, img_points, valid_points, false /* use_depth */);
    error /= ref_points.size();

    ntk_dbg_print(error, 1);
    ntk_dbg_print(new_pose, 2);

    if (error > err_threshold)
        return false;

    if (m_postprocess_with_rgbd_icp)
        optimizeWithRGBDICP(new_pose, image, ref_points, img_points, distances, valid_points);

    return true;
}


struct PointMatchesCompare
{
    PointMatchesCompare(const std::vector<float>& distances)
        : distances(distances)
    {}

    bool operator()(int i1, int i2) const
    {
        return distances[i1] < distances[i2];
    }

    const std::vector<float>& distances;
};

void RelativePoseEstimatorFromRgbFeatures::
optimizeWithRGBDICP(Pose3D& new_pose, const RGBDImage& image,
                    std::vector<Point3f>& ref_points,
                    std::vector<Point3f>& img_points,
                    std::vector<float>& distances,
                    std::vector<bool>& valid_points)
{
    new_pose.toLeftCamera(image.calibration()->depth_intrinsics,
                          image.calibration()->R, image.calibration()->T);

    std::vector<Point3f> clean_ref_points;
    std::vector<Point3f> clean_img_points;
    std::vector<float> clean_distances;
    foreach_idx(i, valid_points)
    {
        if (!valid_points[i])
            continue;
        clean_ref_points.push_back(ref_points[i]);
        clean_img_points.push_back(img_points[i]);
        clean_distances.push_back(distances[i]);
    }

    std::vector<int> best_indices (clean_ref_points.size());
    foreach_idx(i, best_indices) { best_indices[i] = i; }
    std::sort(stl_bounds(best_indices), PointMatchesCompare(clean_distances));
    best_indices.resize(10); // keep to 10 best matches to avoid complete drift.
    std::vector<Point3f> best_ref_points (10);
    std::vector<Point3f> best_img_points (10);
    for (int i = 0; i < best_ref_points.size(); ++i)
    {
        best_ref_points[i] = clean_ref_points[best_indices[i]];
        best_img_points[i] = clean_img_points[best_indices[i]];
    }

    RGBDImage filtered_source_image;
    m_source_image->copyTo(filtered_source_image);

    RGBDImage filtered_target_image;
    m_target_image->copyTo(filtered_target_image);

    OpenniRGBDProcessor processor;
    processor.bilateralFilter(filtered_source_image);
    processor.bilateralFilter(filtered_target_image);
    // processor.computeNormals(filtered_source_image);
    // processor.computeNormals(filtered_target_image);

    MeshGenerator generator;
    generator.setMeshType(MeshGenerator::TriangleMesh);
    generator.setUseColor(true);

    generator.generate(filtered_source_image);
    Mesh source_mesh = generator.mesh();
    source_mesh.computeNormalsFromFaces();

    generator.generate(filtered_target_image);
    Mesh target_mesh = generator.mesh();
    target_mesh.computeNormalsFromFaces();

    pcl::PointCloud<pcl::PointNormal>::Ptr sampled_source_cloud;
    sampled_source_cloud.reset(new pcl::PointCloud<pcl::PointNormal>());

    pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud;
    source_cloud.reset(new pcl::PointCloud<pcl::PointNormal>());

    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud;
    target_cloud.reset(new pcl::PointCloud<pcl::PointNormal>());

    // rgbdImageToPointCloud(*source_cloud, filtered_source_image);
    // rgbdImageToPointCloud(*target_cloud, filtered_target_image);
    meshToPointCloud(*source_cloud, source_mesh);
    meshToPointCloud(*target_cloud, target_mesh);

    const int num_samples = 1000;
    NormalCloudSampler<pcl::PointNormal> sampler;
    sampler.subsample(*source_cloud, *sampled_source_cloud, num_samples);

    RelativePoseEstimatorRGBDICP<pcl::PointNormal> icp_estimator;
    // RelativePoseEstimatorICPWithNormals<pcl::PointNormal> icp_estimator;
    icp_estimator.setVoxelSize(0.001);
    icp_estimator.setDistanceThreshold(0.05);
    icp_estimator.setRANSACOutlierRejectionThreshold(0.02);
    icp_estimator.setMaxIterations(100);

    icp_estimator.setSourceCloud(sampled_source_cloud);
    icp_estimator.setTargetCloud(target_cloud);

    // icp_estimator.setColorFeatures(new_pose, clean_ref_points, clean_img_points);
    icp_estimator.setColorFeatures(new_pose, best_ref_points, best_img_points);
    icp_estimator.setTargetPose(m_target_pose);
    icp_estimator.setInitialSourcePoseEstimate(new_pose);

    bool ok = icp_estimator.estimateNewPose();
    if (!ok)
    {
        ntk_dbg(1) << "RGBD-ICP failed";
        return;
    }
    else
        ntk_dbg(1) << "RGBD-ICP success";

    new_pose = icp_estimator.estimatedSourcePose();

    new_pose.toRightCamera(image.calibration()->depth_intrinsics,
                           image.calibration()->R, image.calibration()->T);
}

struct MatchCompare
{
    bool operator()(const cv::DMatch& m1, const cv::DMatch& m2) const
    {
        return m1.distance < m2.distance;
    }
};

static
void trimMatches(std::vector<cv::DMatch>& matches, float outlier_percent, int min_matches)
{
    if (matches.size() < min_matches)
        return;
    std::sort(stl_bounds(matches), MatchCompare());
    const int num_outliers = std::min(matches.size() * outlier_percent, float(matches.size() - min_matches));
    const int num_inliers = matches.size() - num_outliers;
    std::vector<cv::DMatch> filtered_matches (num_inliers);
    std::copy(matches.begin(), matches.begin() + num_inliers, filtered_matches.begin());
    matches = filtered_matches;
}

bool RelativePoseEstimatorFromRgbFeatures::estimateNewPose()
{
    ntk_assert(m_source_image, "You must call setSourceImage before!");
    ntk_assert(m_target_image, "You must call setTargetImage before!");
    const RGBDImage& image = *m_source_image;

    ntk_ensure(image.mappedDepth().data, "Image must have depth mapping.");

    ntk::TimeCount tc("RelativePoseEstimator", 1);

    if (m_target_features->locations().size() < 1)
    {
        computeTargetFeatures();
    }
    else
    {
        // Recompute the target features pose.
        Pose3D target_rgb_pose = m_target_pose;
        target_rgb_pose.toRightCamera(m_target_image->calibration()->rgb_intrinsics,
                                      m_target_image->calibration()->R, m_target_image->calibration()->T);
        m_target_features->compute3dLocation(target_rgb_pose);
    }

    if (m_source_features->locations().size() < 1)
    {
        m_source_features->extractFromImage(image, m_feature_parameters);
        tc.elapsedMsecs(" -- extract features from Image -- ");
    }

    m_num_matches = 0;

    FeatureSet& image_features = *m_source_features;

    std::vector<cv::DMatch> matches;
    m_target_features->matchWith(image_features, matches, 0.8f*0.8f);
    ntk_dbg_print(matches.size(), 1);
    trimMatches(matches, 0.15f, 100);
    tc.elapsedMsecs(" -- match features -- ");
    ntk_dbg_print(matches.size(), 1);

#ifdef HEAVY_DEBUG
    cv::Mat3b debug_img;
    m_target_features.drawMatches(m_target_image->rgb(), image.rgb(), image_features, matches, debug_img);
    imwrite("/tmp/debug_matches.png", debug_img);
#endif

    m_num_matches = matches.size();

    if (matches.size() < m_min_matches)
        return false;

    m_estimated_pose = m_target_pose;
    m_estimated_pose.toRightCamera(image.calibration()->rgb_intrinsics,
                                   image.calibration()->R, image.calibration()->T);

    // Estimate the relative pose w.r.t the closest view.
    if (!estimateNewPose(m_estimated_pose, image, image_features, matches))
        return false;

    m_estimated_pose.toLeftCamera(image.calibration()->depth_intrinsics,
                                  image.calibration()->R, image.calibration()->T);

    return true;
}

void RelativePoseEstimatorFromRgbFeatures::resetTarget()
{
    m_target_features = toPtr(new FeatureSet);
    m_target_image = 0;
    m_estimated_pose = Pose3D();
}

void RelativePoseEstimatorFromRgbFeatures::setTargetPose(const Pose3D& pose)
{
    m_target_pose = pose;
    m_target_features = toPtr(new FeatureSet);
}

void RelativePoseEstimatorFromRgbFeatures::setTargetImage(const RGBDImage &image)
{
    ntk_ensure(image.calibration(), "Image must be calibrated.");
    m_target_image = &image;
    if (!m_target_pose.isValid())
    {
        m_target_pose = *m_target_image->calibration()->depth_pose;
    }
    m_target_features = toPtr(new FeatureSet);
}

void RelativePoseEstimatorFromRgbFeatures::setSourceImage(const RGBDImage &image)
{
    ntk_ensure(image.calibration(), "Image must be calibrated.");
    super::setSourceImage(image);
    m_source_features = toPtr(new FeatureSet);
}

void RelativePoseEstimatorFromRgbFeatures::computeTargetFeatures()
{
    m_target_features->extractFromImage(*m_target_image, m_feature_parameters);

    Pose3D rgb_pose = m_target_pose;
    rgb_pose.toRightCamera(m_target_image->calibration()->rgb_intrinsics,
                           m_target_image->calibration()->R, m_target_image->calibration()->T);
    m_target_features->compute3dLocation(rgb_pose);
}

void RelativePoseEstimatorFromRgbFeatures::setSourceImage(const RGBDImage &image, ntk::Ptr<FeatureSet> features)
{
    ntk_ensure(image.calibration(), "Image must be calibrated.");
    super::setSourceImage(image);
    m_source_features = features;
}

void RelativePoseEstimatorFromRgbFeatures::setTargetImage(const RGBDImage &image, ntk::Ptr<FeatureSet> features)
{
    setTargetImage(image);
    m_target_features = features;
}

} // ntk
