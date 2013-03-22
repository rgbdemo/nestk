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
