#ifndef RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H
#define RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H

#include "relative_pose_estimator.h"

#include <ntk/image/sift_gpu.h>
#include <ntk/image/feature.h>

namespace ntk
{

/*!
 * Estimate relative 3D pose using feature point detection.
 * Feature matches are computed between the new image and past images,
 * allowing direct estimation of the relative pose.
 */
class RelativePoseEstimatorFromImage : public RelativePoseEstimator
{
public:
  RelativePoseEstimatorFromImage(const FeatureSetParams& params, bool use_icp = false)
   : m_feature_parameters(params), m_use_icp(use_icp), m_incremental_model(true)
  {
    // Force feature extraction to return only features with depth.
    m_feature_parameters.only_features_with_depth = true;
    reset();
  }

  virtual bool estimateNewPose(const RGBDImage& image);
  virtual void reset();
  void setIncrementalModel(bool enable) { m_incremental_model = enable; }

private:
  struct ImageData
  {
    Pose3D depth_pose;
    cv::Mat3b color;
  };

private:
  int newImageIndex() const { return m_image_data.size(); }
  int computeNumMatchesWithPrevious(const RGBDImage& image,
                                    const FeatureSet& features,
                                    std::vector<cv::DMatch>& best_matches);
  bool estimateDeltaPose(Pose3D& new_pose,
                         const RGBDImage& image,
                         const FeatureSet& features,
                         const std::vector<cv::DMatch>& best_matches,
                         int closest_view_index);

  bool optimizeWithICP(const RGBDImage& image, Pose3D& depth_pose, int closest_view_index);

private:
  std::vector < FeatureSet > m_features;
  std::vector< ImageData > m_image_data;
  FeatureSetParams m_feature_parameters;
  bool m_use_icp;
  bool m_incremental_model;
};

} // ntk

#endif // RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H
