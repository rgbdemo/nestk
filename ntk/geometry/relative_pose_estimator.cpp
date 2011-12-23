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

// For QTCreator
// #define NESTK_USE_PCL

#include "relative_pose_estimator.h"

#include <ntk/utils/time.h>
#include <ntk/utils/stl.h>
#include <ntk/stats/histogram.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/opencv_utils.h>
#include <ntk/mesh/mesh.h>

// FIXME: disabled because of conflict between opencv flann and pcl flann.
// using namespace cv;
using cv::Vec3f;
using cv::Point3f;

namespace ntk
{

bool RelativePoseEstimatorFromFile::estimateNewPose(const RGBDImage& image)
{
  ntk_ensure(image.hasDirectory(), "Only works in fake mode!");
  m_current_pose.parseAvsFile((image.directory() + "/relative_pose.avs").c_str());
  return true;
}

bool RelativePoseEstimatorFromDelta::estimateNewPose(const RGBDImage& image)
{
  m_current_pose.applyTransformAfter(m_delta_pose);
  return true;
}

} // ntk
