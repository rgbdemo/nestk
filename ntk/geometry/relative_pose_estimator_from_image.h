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

#ifndef NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H
#define NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H

#include "relative_pose_estimator.h"

//#include <ntk/image/sift_gpu.h>
#include <ntk/image/feature.h>

namespace ntk
{

/*!
 * Estimate relative 3D pose using feature point detection.
 * Feature matches are computed between the new image and past images,
 * allowing direct estimation of the relative pose.
 */
class RelativePoseEstimatorFromRgbFeatures : public RelativePoseEstimatorFromImages
{
public:
    RelativePoseEstimatorFromRgbFeatures(const FeatureSetParams& params)
        : m_feature_parameters(params),
          m_min_matches(10),
          m_num_matches(0)
    {
        // Force feature extraction to return only features with depth.
        m_feature_parameters.only_features_with_depth = true;
        resetTarget();
    }

    virtual void setTargetImage(const RGBDImage &image);
    virtual void setTargetPose(const Pose3D& pose);
    virtual bool estimateNewPose();

    int numMatches() const { return m_num_matches; }

private:
    bool estimateNewPose(Pose3D& new_pose,
                         const RGBDImage& image,
                         const FeatureSet& features,
                         const std::vector<cv::DMatch>& matches);

    void resetTarget();

    void computeTargetFeatures();

private:
    FeatureSet m_target_features;
    FeatureSetParams m_feature_parameters;
    int m_min_matches;
    int m_num_matches;
};

}

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H
