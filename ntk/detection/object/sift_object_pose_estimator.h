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
