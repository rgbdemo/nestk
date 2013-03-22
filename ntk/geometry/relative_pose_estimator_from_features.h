#ifndef NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_FROM_FEATURES_H
#define NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_FROM_FEATURES_H

#include "relative_pose_estimator.h"
#include <ntk/image/feature.h>

namespace ntk
{

/*!
 * Estimate relative 3D pose using 3D feature point correspondence.
 */
class RelativePoseEstimatorFromFeatures : public RelativePoseEstimator
{
public:
    RelativePoseEstimatorFromFeatures()
    {
    }

    void setTargetFeatures(const FeatureSet& target) {}
    void setSourceFeatures(const FeatureSet& source) {}

    virtual bool estimateNewPose();

    int numMatches() const { return m_num_matches; }

private:
    int m_num_matches;
};

}

#endif // NTK_GEOMETRY_RELATIVE_POSE_ESTIMATOR_FROM_IMAGE_H
