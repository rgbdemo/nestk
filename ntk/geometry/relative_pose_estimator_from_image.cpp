#include "relative_pose_estimator_from_image.h"

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
    const float err_threshold = 0.05;

    std::vector<Point3f> ref_points;
    std::vector<Point3f> img_points;
    std::vector<cv::KeyPoint> ref_keypoints;
    std::vector<cv::KeyPoint> img_keypoints;

    foreach_idx(i, matches)
    {
        const cv::DMatch& m = matches[i];
        const FeaturePoint& ref_loc = m_target_features.locations()[m.trainIdx];
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

    std::vector<bool> valid_points;
    double error = rms_optimize_ransac(new_pose, ref_points, img_points, valid_points);

    ntk_dbg_print(error, 1);
    ntk_dbg_print(new_pose, 2);

    if (error < err_threshold)
        return true;
    else
        return false;
}

bool RelativePoseEstimatorFromRgbFeatures::estimateNewPose()
{
    ntk_assert(m_source_image, "You must call setSourceImage before!");
    ntk_assert(m_target_image, "You must call setTargetImage before!");
    const RGBDImage& image = *m_source_image;

    ntk::TimeCount tc("RelativePoseEstimator", 1);

    if (m_target_features.locations().size() < 1)
        computeTargetFeatures();

    m_num_matches = 0;

    ntk_ensure(image.mappedDepth().data, "Image must have depth mapping.");
    FeatureSet image_features;
    image_features.extractFromImage(image, m_feature_parameters);
    tc.elapsedMsecs(" -- extract features from Image -- ");

    std::vector<cv::DMatch> matches;
    m_target_features.matchWith(image_features, matches, 0.8*0.8);
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
    m_target_features = FeatureSet();
    m_target_image = 0;
    m_estimated_pose = Pose3D();
}

void RelativePoseEstimatorFromRgbFeatures::setTargetPose(const Pose3D& pose)
{
    m_target_pose = pose;
    m_target_features = FeatureSet();
}

void RelativePoseEstimatorFromRgbFeatures::setTargetImage(const RGBDImage &image)
{
    ntk_ensure(image.calibration(), "Image must be calibrated.");
    m_target_image = &image;
    if (!m_target_pose.isValid())
    {
        m_target_pose = *m_target_image->calibration()->depth_pose;
    }
    m_target_features = FeatureSet();
}

void RelativePoseEstimatorFromRgbFeatures::computeTargetFeatures()
{
    m_target_features.extractFromImage(*m_target_image, m_feature_parameters);

    Pose3D rgb_pose = m_target_pose;
    rgb_pose.toRightCamera(m_target_image->calibration()->rgb_intrinsics,
                           m_target_image->calibration()->R, m_target_image->calibration()->T);
    m_target_features.compute3dLocation(rgb_pose);
}

} // ntk
