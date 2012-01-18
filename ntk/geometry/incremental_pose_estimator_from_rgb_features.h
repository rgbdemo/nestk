/**
 * This file is part of the nestk library.
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
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_FROM_RGB_FEATURES_H
#define NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_FROM_RGB_FEATURES_H

#include "incremental_pose_estimator.h"
#include "relative_pose_estimator.h"
#include <ntk/image/feature.h>

namespace ntk
{

/*!
 * Estimate relative 3D pose using feature point detection.
 * Feature matches are computed between the new image and past images,
 * allowing direct estimation of the relative pose.
 * NOTE: this class is a big mess, only here to keep a reference implementation.
 */
class IncrementalPoseEstimatorFromRgbFeatures : public IncrementalPoseEstimatorFromImage
{
public:
    IncrementalPoseEstimatorFromRgbFeatures(const FeatureSetParams& params, bool use_icp = false)
        : m_feature_parameters(params), m_use_icp(use_icp), m_incremental_model(true)
    {
        // Force feature extraction to return only features with depth.
        m_feature_parameters.only_features_with_depth = true;
        reset();
    }

    IncrementalPoseEstimatorFromRgbFeatures(const IncrementalPoseEstimatorFromRgbFeatures& rhs)
        : m_feature_parameters(rhs.m_feature_parameters),
          m_use_icp(rhs.m_use_icp),
          m_incremental_model(rhs.m_incremental_model)
    {
        reset();
    }

    virtual bool estimateCurrentPose();
    virtual void reset();
    void setIncrementalModel(bool enable) { m_incremental_model = enable; }

private:
    struct ImageData
    {
#ifdef NESTK_USE_PCL
        // Subsampled original point cloud, in original frame.
        pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud;
#endif
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

#ifdef NESTK_USE_PCL
    bool optimizeWithICP(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_source,
                         Pose3D& depth_pose,
                         int closest_view_index);
#endif

private:
    std::vector < FeatureSet > m_features;
    std::vector< ImageData > m_image_data;
    FeatureSetParams m_feature_parameters;
    bool m_use_icp;
    bool m_incremental_model;
};

} // ntk

#endif // NTK_GEOMETRY_INCREMENTAL_POSE_ESTIMATOR_FROM_RGB_FEATURES_H
