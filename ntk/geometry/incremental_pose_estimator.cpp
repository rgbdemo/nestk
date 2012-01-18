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

