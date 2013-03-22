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

#ifndef SIFTOBJECTPOSEESTIMATOR_H
#define SIFTOBJECTPOSEESTIMATOR_H

#include <ntk/core.h>

#include "object_pose.h"
#include "sift_point_match.h"

namespace ntk
{

  class ObjectDetectorData;
  class SiftParameters;

  class SiftObjectPoseEstimator
  {
  public:
    SiftObjectPoseEstimator(const SiftParameters& params) : m_params(params) {}

  public:
    bool optimize(ObjectPose& pose,
                  SiftPointMatchConstPtrSet& matches,
                  const ObjectDetectorData& finder_data);

    bool computeFastPose(ObjectPose& pose,
                         SiftPointMatchConstPtrSet& matches,
                         const ObjectDetectorData& finder_data);

  private:
    const SiftParameters& m_params;
  };

} // avs

#endif // SIFTOBJECTPOSEESTIMATOR_H
