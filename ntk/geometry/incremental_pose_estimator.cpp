
#include "incremental_pose_estimator.h"

bool ntk::IncrementalPoseEstimatorFromFile::estimateCurrentPose()
{
    ntk_ensure(m_new_image.hasDirectory(), "Only works in fake mode!");
    m_current_pose.parseAvsFile((m_new_image.directory() + "/relative_pose.avs").c_str());
    return true;
}

bool ntk::IncrementalPoseEstimatorFromDelta::estimateCurrentPose()
{
    m_current_pose.applyTransformAfter(m_delta_pose);
    return true;
}

